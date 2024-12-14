#include "m3_exception.h"
#include "m3_compile.h"
#include "m3_env.h"
#include "m3_bind.h"

#include "m3_validate.h"


static M3Result
ValidateTables(IM3Module module)
{
    u32 lim_min = module->table0SizeMin;
    i32 lim_max = module->table0SizeMax;

    if (
        ( (lim_max < 0) && (lim_max != -1) )
        || ( (lim_max >= 0) && ((u32)lim_max < lim_min) )
    )
    {
        return m3Err_wasmMalformed;
    }

    return m3Err_none;  //  wasm3 supports only one table
}

static M3Result
ValidateMemory(IM3Module module)
{
    u32 lim_min = module->memoryInfo.initPages;
    i32 lim_max = module->memoryInfo.imaxPages;
    if (
        ( (lim_max < 0) && (lim_max != -1) )
        || ( (lim_max >= 0) && ((u32)lim_max < lim_min) )
        || (lim_max > 65536)
    )
    {
        return m3Err_wasmMalformed;
    }
    
    return m3Err_none;  // parser already counted memories
}

static M3Result
ValidateGlobals(IM3Module module)
{
    M3Result result = m3Err_none;
    u32 num_globals = module->numGlobals;
    for (u32 i = 0; i < num_globals; i++)
    {
        M3Global global = module->globals[i];
        if (global.imported)
        {
            continue;
        }
        bytes_t i_bytes = global.initExpr;
        bytes_t end = i_bytes + global.initExprSize;
        u8 op_byte;
        _( ReadLEB_u7(&op_byte, &i_bytes, end) );
        u8 init_expr_type;
        switch (op_byte)
        {
            case c_waOp_i32_const:
            {
                init_expr_type = c_m3Type_i32;
                break;
            }
            case c_waOp_i64_const:
            {
                init_expr_type = c_m3Type_i64;
                break;
            }
            case c_waOp_f32_const:
            {
                init_expr_type = c_m3Type_f32;
                break;
            }
            case c_waOp_f64_const:
            {
                init_expr_type = c_m3Type_f64;
                break;
            }
            case c_waOp_getGlobal:
            {
                u32 global_get_idx;
                _( ReadLEB_u32(&global_get_idx, &i_bytes, end) );
                if (global_get_idx >= module->numGlobImports)
                {
                    _throw(m3Err_wasmMalformed)
                }
                init_expr_type = module->globals[global_get_idx].type;
                break;
            }
            default:
            {
                _throw(m3Err_wasmMalformed);
            }
        }
        if (global.type != init_expr_type)
        {
            _throw(m3Err_wasmMalformed);
        }
    }
    
    _catch: return result;
}

static M3Result
ValidateStart(IM3Module module)
{
    M3Result result = m3Err_none;
    i32 start_idx = module->startFunction;

    if (start_idx == -1)
    {
        return m3Err_none;
    }

    _throwif(m3Err_wasmMalformed, start_idx < 0);
    _throwif(m3Err_wasmMalformed, (u32)start_idx >= module->numFunctions);

    M3Function start = module->functions[start_idx];
    IM3FuncType start_type = start.funcType;

    _throwif(
        m3Err_wasmMalformed,
        (start_type->numArgs != 0 || start_type->numRets != 0)
        );

    _catch: return result;
}

static M3Result
ValidateElem(IM3Module module)
{
    //  assert: 0, i32.const, (vec u32) of function indices
    //
    M3Result result = m3Err_none;
    bytes_t i_bytes = module->elementSection;
    bytes_t end = module->elementSectionEnd;
    for (u32 i = 0; i < module->numElementSegments; ++i)
    {
        u8 label;
        _( Read_u8(&label, &i_bytes, end) );
        _throwif(m3Err_wasmMalformed, label);  // elem tag 0x0
        _throwif("only table idx != 0", module->table0Idx != 0);
        
        i32 offset;
        _( EvaluateExpression (module, &offset, c_m3Type_i32, &i_bytes, end) );
        _throwif (m3Err_wasmMalformed, offset < 0);

        u32 numElements;
        _( ReadLEB_u32(&numElements, &i_bytes, end) );

        while (numElements--)
        {
            u32 func_idx;
            _( ReadLEB_u32(&func_idx, &i_bytes, end) );

            _throwif(m3Err_wasmMalformed, func_idx >= module->numFunctions);
        }
    }

    _throwif(m3Err_wasmMalformed, i_bytes != end);

    _catch: return result;
}

static M3Result
ValidateDatacnt(IM3Module module)
{
    i32 data_cnt = module->dataCnt;
    if (data_cnt == -1)
    {
        return m3Err_none;
    }
    if ( (data_cnt < 0) || ((u32)data_cnt != module->numDataSegments) )
    {
        return m3Err_wasmMalformed;
    }

    return m3Err_none;
}

typedef enum {
    bk_loop,
    bk_if,
    bk_block
} BlockKind;

typedef struct Frame {
    BlockKind kind;
    bool is_true_branch;
    u32 base_i;  //  start of the type stack for this frame
    IM3FuncType blocktype;
} Frame;

//  forward declare mutually recursive function
static M3Result
SkipImmediateArgs(m3opcode_t op,
                  i8 numArgsImmediate,
                  bytes_t *io_bytes,
                  cbytes_t end,
                  IM3Module module
);

static M3Result
SkipPastEndElse(bool* is_else,
                bytes_t *io_bytes,
                cbytes_t end,
                IM3Module module)
{
    M3Result result = m3Err_none;
    while (1)
    {
        m3opcode_t op;
        u8 op_short;
        _( Read_u8(&op_short, io_bytes, end) );
        if (op_short == 0xFC)
        {
            u8 fc_rest;
            _( Read_u8(&fc_rest, io_bytes, end) );
            op = (op_short << 8) | fc_rest;
        }
        else
        {
            op = op_short;
        }

        if (op == c_waOp_else)
        {
            *is_else = 1;
            return m3Err_none;
        }
        else if (op == c_waOp_end)
        {
            *is_else = 0;
            return m3Err_none;
        }
        
        IM3OpInfo info = GetOpInfo(op);
        _throwif("no info", !info);

        i8 imm_args = info->numArgsImmediate;
        if (imm_args != 0)
        {
            _( SkipImmediateArgs(op, imm_args, io_bytes, end, module) );
        }
    }

    _catch: return result;
}

static M3Result
SkipImmediateArgs(m3opcode_t op,
                  i8 numArgsImmediate,
                  bytes_t *io_bytes,
                  cbytes_t end,
                  IM3Module module)
{
    M3Result result = m3Err_none;

    if (numArgsImmediate >= 0)
    {
        u32 sink;
        while ((numArgsImmediate--) > 0)
        {
            _( ReadLEB_u32(&sink, io_bytes, end) );
        }
    }
    else if (numArgsImmediate == -2)
    {
        //  memarg
        u32 align, offset;
        _( ReadLEB_u32(&align, io_bytes, end) );
        _( ReadLEB_u32(&offset, io_bytes, end) );

        u32 byte_width;
        switch (op)
        {
            case c_waOp_i32_load:
            case c_waOp_f32_load:
            case c_waOp_i32_store:
            case c_waOp_f32_store:
            case c_waOp_i64_load32_s:
            case c_waOp_i64_load32_u:
            case c_waOp_i64_store32:
            {
                byte_width = 4;
                break;
            }
            case c_waOp_i64_load:
            case c_waOp_f64_load:
            case c_waOp_i64_store:
            case c_waOp_f64_store:
            {
                byte_width = 8;
                break;
            }
            case c_waOp_i32_load8_s:
            case c_waOp_i32_load8_u:
            case c_waOp_i64_load8_s:
            case c_waOp_i64_load8_u:
            case c_waOp_i32_store8:
            case c_waOp_i64_store8:
            {
                byte_width = 1;
                break;
            }
            case c_waOp_i32_load16_s:
            case c_waOp_i32_load16_u:
            case c_waOp_i64_load16_s:
            case c_waOp_i64_load16_u:
            case c_waOp_i32_store16:
            case c_waOp_i64_store16:
            {
                byte_width = 2;
                break;
            }
            default:
            {
                _throw("can't calculate width");
            }
        }

        _throwif("misaligned", ((1 << align) > byte_width));

    }
    else
    {
        switch (op)
        {
            case c_waOp_i32_const:
            {
                i32 sink;
                _( ReadLEB_i32(&sink, io_bytes, end) );
                break;
            }
            case c_waOp_i64_const:
            {
                i64 sink;
                _( ReadLEB_i64(&sink, io_bytes, end) );
                break;
            }
            case c_waOp_f32_const:
            {
                f32 sink;
                _( Read_f32(&sink, io_bytes, end) );
                break;
            }
            case c_waOp_f64_const:
            {
                f64 sink;
                _( Read_f64(&sink, io_bytes, end) );
                break;
            }
            case c_waOp_block:
            case c_waOp_loop:
            {
                i64 sink;
                _( ReadLebSigned (& sink, 33, io_bytes, end) );
                bool is_else;
                _( SkipPastEndElse(&is_else, io_bytes, end, module) );
                _throwif("unbalanced else", is_else);
                break;
            }
            case c_waOp_if:
            {
                i64 sink;
                _( ReadLebSigned (& sink, 33, io_bytes, end) );
                bool is_else;
                _( SkipPastEndElse(&is_else, io_bytes, end, module) );
                if (is_else)
                {
                    _( SkipPastEndElse(&is_else, io_bytes, end, module) );
                    _throwif("unbalanced else", is_else);
                }
                break;
            }
            case c_waOp_branchTable:
            {
                u32 n_depths;
                _( ReadLEB_u32(&(n_depths), io_bytes, end) );
                n_depths++;

                for (u32 i = 0; i < n_depths; i++)
                {
                    u32 sink;
                    _( ReadLEB_u32(&sink, io_bytes, end) );
                }
                break;
            }
            case c_waOp_memorySize:
            case c_waOp_memoryGrow:
            case c_waOp_memoryFill:
            {
                u32 mem_idx;
                _( ReadLEB_u32(&(mem_idx), io_bytes, end) );
                _throwif("memidx > 0", mem_idx);
                _throwif("no memory", !module->memoryInfo.hasMemory);
                break;
            }
            case c_waOp_memoryCopy:
            {
                u32 mem_idx1, mem_idx2;
                _( ReadLEB_u32(&(mem_idx1), io_bytes, end) );
                _( ReadLEB_u32(&(mem_idx2), io_bytes, end) );
                _throwif("memidx > 0", mem_idx1);
                _throwif("memidx > 0", mem_idx2);
                _throwif("no memory", !module->memoryInfo.hasMemory);
                break;
            }
            case c_waOp_call_indirect:
            {
                u32 type_idx, table_idx;
                _( ReadLEB_u32(&(type_idx), io_bytes, end) );
                _( ReadLEB_u32(&(table_idx), io_bytes, end) );
                _throwif("weird table", table_idx != module->table0Idx);
                _throwif(
                    "type idx out of range",
                    type_idx >= module->numFuncTypes
                );
                break;
            }
            // else, end are skipped as part of an expression
            default:
            {
                _throw("unbalnced else/end");
            }
        }
    }

    _catch: return result;
}

static M3Result
ParseBlockType(IM3FuncType * o_blockType,
               bytes_t *io_bytes,
               cbytes_t end,
               IM3Module module)
{
    M3Result result = m3Err_none;

    i64 type;
    _( ReadLebSigned (& type, 33, io_bytes, end) );

    if (type < 0)
    {
        u8 valueType;
        _( NormalizeType (&valueType, type) );
        *o_blockType = module->environment->retFuncTypes[valueType];
    }

    _catch: return result;
}

static M3Result
ValidateFuncBody(M3Function function, IM3Module module)
{
    M3Result result = m3Err_none;

    u8 local_types[d_m3MaxSaneLocalCount];
    u32 total_locals;
    bytes_t io_bytes = function.wasm;
    cbytes_t end = function.wasmEnd;

    {   //  skip size
        u32 size;
        _( ReadLEB_u32(&size, &io_bytes, end) );
    }
    {
        IM3FuncType type = function.funcType;
        u32 j = 0;
        u32 numArgs = type->numArgs;
        u32 numRets = type->numRets;
        u8 *types = type->types;
        for (u32 arg_i = numRets; arg_i < (numRets + numArgs); arg_i++)
        {
            local_types[j] = types[arg_i];
            j++;
        }
        u32 num_local_blocks;
        _( ReadLEB_u32(&num_local_blocks, &io_bytes, end) );

        for (u32 i = 0; i < num_local_blocks; i++)
        {
            u32 num_locals;
            i8 waType;
            u8 type;
            _( ReadLEB_u32(&num_locals, &io_bytes, end) );
            _( ReadLEB_i7(&waType, &io_bytes, end) );
            _( NormalizeType (&type, waType) );
            for (u32 k = 0; k < num_locals; k++)
            {
                _throwif("insane locals", (j > d_m3MaxSaneLocalCount));
                local_types[j] = type;
                j++;
            }
        }
        total_locals = j;
    }

    u8 type_stack[d_m3MaxSaneTypeStackSize] = {0};
    Frame frames[d_m3MaxSaneFrameDepth] = {0};

    u32 type_i = 0, frame_i = 0;
    Frame frame_init = {bk_block, 0, 0, function.funcType};
    frames[frame_i++] = frame_init;
    while (frame_i)
    {
        m3opcode_t op;
        u8 op_short;
        _( Read_u8(&op_short, &io_bytes, end) );
        if (op_short == 0xFC)
        {
            u8 fc_rest;
            _( Read_u8(&fc_rest, &io_bytes, end) );
            op = (op_short << 8) | fc_rest;
        }
        else
        {
            op = op_short;
        }

        IM3OpInfo info = GetOpInfo(op);
        _throwif("no info", !info);
        if (info->typeSignature)
        {
            ccstr_t signature = info->typeSignature;
            Frame now = frames[frame_i - 1];
            {
                u8 i_arg = info->numArgs;
                while (i_arg--)
                {
                    _throwif("stack empty", (type_i == now.base_i));

                    u8 type_id = ConvertTypeCharToTypeId(signature[i_arg]);
                    _throwif(
                        "type mismatch",
                        (type_stack[--type_i] != type_id)
                    );
                }
            }
            u8 num_tot = info->numArgs + info->numRets;
            for (u8 i_ret = info->numArgs; i_ret < num_tot; i_ret++ )
            {
                _throwif(
                    "insane stack",
                    (type_i >= d_m3MaxSaneTypeStackSize)
                );
                u8 type = ConvertTypeCharToTypeId(signature[i_ret]);
                type_stack[type_i++] = type;
            }
            
            i8 imm_args = info->numArgsImmediate;
            if (imm_args != 0)
            {
                _(SkipImmediateArgs(op, imm_args, &io_bytes, end, module));
            }
        }
        else
        {
            switch (op)
            {
                // "dynamic" types
                #define READ_DEPTH_TEST                                         \
                    do {                                                        \
                        u32 depth;                                              \
                        _( ReadLEB_u32(&depth, &io_bytes, end) );               \
                        _throwif("out of depth", (depth >= frame_i));           \
                        Frame frame = frames[frame_i - depth - 1];              \
                        Frame now = frames[frame_i - 1];                        \
                        IM3FuncType blocktype = frame.blocktype;                \
                        if (frame.kind == bk_loop)                              \
                        {                                                       \
                            u16 numArgs = blocktype->numArgs;                   \
                            u16 numRets = blocktype->numRets;                   \
                            u16 arg_i = numArgs + numRets;                      \
                            u32 type_i_temp = type_i;                           \
                            u8 *types = blocktype->types;                       \
                            _throwif(                                           \
                                "weird stack",                                  \
                                type_i_temp < now.base_i                        \
                            );                                                  \
                            while ((arg_i--) > numRets)                         \
                            {                                                   \
                                _throwif(                                       \
                                    "out of stack",                             \
                                    type_i_temp == now.base_i                   \
                                );                                              \
                                _throwif(                                       \
                                    "branch type mismatch",                     \
                                    type_stack[--type_i_temp] != types[arg_i]   \
                                );                                              \
                            }                                                   \
                        }                                                       \
                        else                                                    \
                        {                                                       \
                            u16 ret_i = blocktype->numRets;                     \
                            u32 type_i_temp = type_i;                           \
                            u8 *types = blocktype->types;                       \
                            _throwif(                                           \
                                "weird stack",                                  \
                                type_i_temp < now.base_i                        \
                            );                                                  \
                            while ((ret_i--) > 0)                               \
                            {                                                   \
                                _throwif(                                       \
                                    "out of stack",                             \
                                    type_i_temp == now.base_i                   \
                                );                                              \
                                _throwif(                                       \
                                    "branch type mismatch",                     \
                                    type_stack[--type_i_temp] != types[ret_i]   \
                                );                                              \
                            }                                                   \
                        }                                                       \
                    } while (0)

                case c_waOp_nop:
                {
                    continue;
                }
                case c_waOp_branchIf:
                {
                    Frame now = frames[frame_i - 1];
                    _throwif(
                        "branch if no i32",
                        ( (type_i <= now.base_i)
                          || (type_stack[--type_i] != c_m3Type_i32)
                        )
                    );
                    
                    READ_DEPTH_TEST;

                    continue;
                }
                case c_waOp_call:
                {
                    u32 func_idx;
                    _( ReadLEB_u32(&func_idx, &io_bytes, end) );
                    _throwif(
                        "call idx out of range",
                        (func_idx >= module->numFunctions)
                    );

                    IM3FuncType functype = module->functions[func_idx].funcType;
                    u16 numArgs = functype->numArgs;
                    u16 numRets = functype->numRets;
                    u8 *types = functype->types;
                    u16 arg_i = numArgs + numRets;
                    Frame now = frames[frame_i - 1];
                    _throwif("weird stack", type_i < now.base_i);

                    while ((arg_i--) > numRets)
                    {
                        _throwif("out of stack", type_i == now.base_i);
                        _throwif(
                            "call wrong type",
                            (type_stack[--type_i] != types[arg_i])
                        );
                    }
                    _throwif(
                        "insane stack",
                        (type_i >= d_m3MaxSaneTypeStackSize
                                   - d_m3MaxSaneFunctionArgRetCount)
                    );
                    for (u16 ret_i = 0; ret_i < numRets; ret_i++)
                    {
                        type_stack[type_i++] = types[ret_i];
                    }
                    
                    continue;
                }
                case c_waOp_call_indirect:
                {
                    u32 type_idx, table_idx;
                    Frame now = frames[frame_i - 1];
                    _( ReadLEB_u32(&type_idx, &io_bytes, end) );
                    _( ReadLEB_u32(&table_idx, &io_bytes, end) );

                    _throwif("weird table", table_idx != module->table0Idx);  // only one table supported
                    _throwif("call indirect empty stack", type_i <= now.base_i);
                    _throwif(
                        "call indirect idx wrong type",
                        (type_stack[--type_i] != c_m3Type_i32)
                    );
                    _throwif(
                        "call indirect type idx out of range",
                        (type_idx >= module->numFuncTypes)
                    );
                    IM3FuncType functype = module->funcTypes[type_idx];
                    u16 numArgs = functype->numArgs;
                    u16 numRets = functype->numRets;
                    u8 *types = functype->types;
                    u16 arg_i = numArgs + numRets;

                    _throwif("weird stack", type_i < now.base_i);
                    while ((arg_i--) > numRets)
                    {
                        _throwif(
                            "call indirect out of stack",
                            type_i == now.base_i
                        );
                        _throwif(
                            "call indirect wrong type",
                            (type_stack[--type_i] != types[arg_i])
                        );
                    }
                    _throwif(
                        "stack insane",
                        (type_i >= d_m3MaxSaneTypeStackSize
                                   - d_m3MaxSaneFunctionArgRetCount)
                    );

                    for (u16 ret_i = 0; ret_i < numRets; ret_i++)
                    {
                        type_stack[type_i++] = types[ret_i];
                    }
                    
                    continue;
                }
                case c_waOp_drop:
                {
                    Frame now = frames[frame_i - 1];
                    _throwif("nothing to drop", (type_i--) <= now.base_i);
                    continue;
                }
                case c_waOp_select:
                {
                    Frame now = frames[frame_i - 1];
                    _throwif("select out of stack", type_i < 3 + now.base_i);
                    _throwif(
                        "select not i32",
                        (type_stack[--type_i] != c_m3Type_i32)
                    );
                    u8 t1 = type_stack[--type_i];
                    u8 t2 = type_stack[type_i - 1];

                    _throwif("select type mismatch", (t1 != t2));

                    continue;
                }
                case c_waOp_getLocal:
                {
                    u32 local_idx;
                    _( ReadLEB_u32(&local_idx, &io_bytes, end) );

                    _throwif(
                        "get local idx out of range",
                        local_idx >= total_locals
                    );
                    _throwif(
                        "stack insane",
                        (type_i >= d_m3MaxSaneTypeStackSize)
                    );
                    type_stack[type_i++] = local_types[local_idx];

                    continue;
                }
                case c_waOp_setLocal:
                {
                    Frame now = frames[frame_i - 1];
                    u32 local_idx;
                    _( ReadLEB_u32(&local_idx, &io_bytes, end) );

                    _throwif(
                        "get local idx out of range",
                        local_idx >= total_locals
                    );

                    _throwif("set local empty stack", type_i <= now.base_i);

                    _throwif(
                        "set local wrong type",
                        (type_stack[--type_i] != local_types[local_idx])
                    );

                    continue;
                }
                case c_waOp_teeLocal:
                {
                    Frame now = frames[frame_i - 1];
                    u32 local_idx;
                    _( ReadLEB_u32(&local_idx, &io_bytes, end) );

                    _throwif(
                        "tee local idx out of range",
                        local_idx >= total_locals
                    );

                    _throwif("tee empty ", type_i <= now.base_i);

                    _throwif(
                        "tee stack empty",
                        (type_stack[type_i-1] != local_types[local_idx])
                    );

                    continue;
                }
                case c_waOp_getGlobal:
                {
                    u32 global_idx;
                    _( ReadLEB_u32(&global_idx, &io_bytes, end) );

                    _throwif(
                        "global idx out of range",
                        (global_idx >= module->numGlobals)
                    );

                    M3Global global = module->globals[global_idx];
                    _throwif(
                        "insane stack",
                        (type_i >= d_m3MaxSaneTypeStackSize)
                    );
                    type_stack[type_i++] = global.type;

                    continue;
                }
                case c_waOp_setGlobal:
                {
                    Frame now = frames[frame_i - 1];
                    u32 global_idx;
                    _( ReadLEB_u32(&global_idx, &io_bytes, end) );

                    _throwif(
                        "global idx out of range",
                        (global_idx >= module->numGlobals)
                    );

                    M3Global global = module->globals[global_idx];

                    _throwif("global not mutable", !global.isMutable);

                    _throwif("global.set empty stack", type_i <= now.base_i);

                    _throwif(
                        "global.set wrong type",
                        (type_stack[--type_i] != global.type)
                    );

                    continue;
                }

                // control: leave frame

                #define LEAVE_FRAME                                             \
                    do {                                                        \
                        bool is_else;                                           \
                        _( SkipPastEndElse(&is_else, &io_bytes, end, module) ); \
                        if (is_else)                                            \
                        {                                                       \
                            Frame* now = &frames[frame_i - 1];                  \
                            bool frame_is_if = (bk_if == now->kind);            \
                            bool frame_true_branch = now->is_true_branch;       \
                            _throwif(                                           \
                                "unbalanced else",                              \
                                !(frame_is_if && frame_true_branch)             \
                            );                                                  \
                            now->is_true_branch = 0;                            \
                            type_i = now->base_i;                               \
                            u16 numArgs = now->blocktype->numArgs;              \
                            u16 numRets = now->blocktype->numRets;              \
                            u16 numTot = numRets + numArgs;                     \
                            u8 *types = now->blocktype->types;                  \
                            _throwif(                                           \
                                "insane stack",                                 \
                                (type_i >= d_m3MaxSaneTypeStackSize             \
                                           - d_m3MaxSaneFunctionArgRetCount)    \
                            );                                                  \
                            for (u32 arg_i = numRets; arg_i < numTot; arg_i++)  \
                            {                                                   \
                                type_stack[type_i++] = types[arg_i];            \
                            }                                                   \
                        }                                                       \
                        else                                                    \
                        {                                                       \
                            Frame popped = frames[--frame_i];                   \
                            type_i = popped.base_i;                             \
                            u32 numRets = popped.blocktype->numRets;            \
                            u32 numArgs = popped.blocktype->numArgs;            \
                            u8 *types = popped.blocktype->types;                \
                            _throwif(                                           \
                                "insane stack",                                 \
                                (type_i >= d_m3MaxSaneTypeStackSize             \
                                           - d_m3MaxSaneFunctionArgRetCount)    \
                            );                                                  \
                            for (u32 ret_i = 0; ret_i < numRets; ret_i++)       \
                            {                                                   \
                                type_stack[type_i++] = types[ret_i];            \
                            }                                                   \
                        }                                                       \
                    } while (0)

                case c_waOp_unreachable:
                {
                    LEAVE_FRAME;
                    continue;
                }
                case c_waOp_else:
                {
                    Frame *now = &frames[frame_i-1];
                    bool frame_is_if = (bk_if == now->kind);
                    bool frame_true_branch = now->is_true_branch;
                    _throwif(
                        "unbalanced else",
                        !(frame_is_if && frame_true_branch)
                    );
                    _throwif("weird stack", (type_i < now->base_i));
                    u16 numArgs = now->blocktype->numArgs;
                    u16 numRets = now->blocktype->numRets;
                    u16 numTot = numArgs + numRets;
                    u8* types = now->blocktype->types;
                    u16 ret_i = numRets;
                    while (ret_i--)
                    {
                        _throwif("else empty stack", type_i == now->base_i);
                        _throwif(
                            "else wrong type",
                            types[ret_i] != type_stack[--type_i]
                        );
                    }
                    _throwif("else wrong count", (type_i != now->base_i));
                    now->is_true_branch = 0;
                    _throwif(
                        "insane stack",
                        (type_i >= d_m3MaxSaneTypeStackSize
                                    - d_m3MaxSaneFunctionArgRetCount)
                    );
                    for (u32 arg_i = numRets; arg_i < numTot; arg_i++)
                    {                                                 
                        type_stack[type_i++] = types[arg_i];          
                    }                                                 

                    continue;
                }
                case c_waOp_end:
                {
                    Frame popped = frames[--frame_i];
                    _throwif("weird stack", (type_i < popped.base_i));
                    u16 numRets = popped.blocktype->numRets;
                    u8* types = popped.blocktype->types;
                    u16 ret_i = numRets;
                    u32 type_i_temp = type_i;
                    while (ret_i--)
                    {
                        _throwif(
                            "end out of stack",
                            type_i_temp == popped.base_i
                        );
                        _throwif(
                            "end wrong type",
                            types[ret_i] != type_stack[--type_i_temp]
                        );
                    }
                    _throwif(
                        "end wrong count",
                        (type_i_temp != popped.base_i)
                    );

                    continue;
                }
                case c_waOp_branch:
                {
                    READ_DEPTH_TEST;

                    LEAVE_FRAME;

                    continue;
                }
                case c_waOp_branchTable:
                {
                    _throwif(
                        "branch table no i32 idx",
                        ( (type_i <= frames[frame_i - 1].base_i)
                          || (c_m3Type_i32 != type_stack[--type_i]) )
                    );

                    u32 n_depths;
                    _( ReadLEB_u32(&(n_depths), &io_bytes, end) );
                    n_depths++;

                    for (u32 i = 0; i < n_depths; i++)
                    {
                        READ_DEPTH_TEST;
                    }

                    LEAVE_FRAME;
                    continue;
                }
                case c_waOp_return:
                {
                    Frame frame = frames[0];
                    IM3FuncType blocktype = frame.blocktype;
                    u16 ret_i = blocktype->numRets;
                    u32 type_i_temp = type_i;
                    u8 *types = blocktype->types;
                    Frame now = frames[frame_i - 1];
                    _throwif("weird stack", type_i_temp < now.base_i);

                    while ((ret_i--) > 0)
                    {
                        _throwif(
                            "return out of stack",
                            type_i_temp == now.base_i
                        );
                        _throwif(
                            "return wrong type",
                            type_stack[--type_i_temp] != types[ret_i]
                        );
                    }

                    LEAVE_FRAME;
                    continue;
                }

                // control: enter frame
                #define ENTER_FRAME(KIND, IS_TRUE_BRANCH)                       \
                    do                                                          \
                    {                                                           \
                        IM3FuncType blocktype;                                  \
                        _( ParseBlockType(&blocktype, &io_bytes, end, module) );\
                        u16 numArgs = blocktype->numArgs;                       \
                        u16 numRets = blocktype->numRets;                       \
                        u8 *types = blocktype->types;                           \
                        u16 arg_i = numArgs + numRets;                          \
                        Frame now = frames[frame_i - 1];                        \
                        _throwif("weird stack", type_i < now.base_i);           \
                        while ((arg_i--) > numRets)                             \
                        {                                                       \
                            _throwif(                                           \
                                "enter frame out of stack",                     \
                                type_i == now.base_i                            \
                            );                                                  \
                            _throwif(                                           \
                                "enter frame wrong type",                       \
                                type_stack[--type_i] != types[arg_i]            \
                            );                                                  \
                        }                                                       \
                        Frame new = {KIND, IS_TRUE_BRANCH, type_i, blocktype};  \
                        frames[frame_i++] = new;                                \
                        type_i += numArgs;                                      \
                    } while (0)

                case c_waOp_block:
                {
                    ENTER_FRAME(bk_block, 0);
                    continue;
                }
                case c_waOp_loop:
                {
                    ENTER_FRAME(bk_loop, 0);
                    continue;
                }
                case c_waOp_if:
                {
                    _throwif(
                        "if no flag",
                        ( (type_i <= frames[frame_i-1].base_i)
                          || (type_stack[--type_i] != c_m3Type_i32) )
                    );

                    ENTER_FRAME(bk_if, 1);
                    continue;
                }

                // weird: not covered by the formal spec
                case c_waOp_return_call:
                case c_waOp_return_call_indirect:
                {
                    _throw("not implemented");
                }

                default:
                {
                    _throw("not recognized");
                }
            }
        }
    }

    _throwif(m3Err_wasmMalformed, (io_bytes != end));

    _catch: return result;
}


static M3Result
ValidateCode(IM3Module module)
{
    M3Result result = m3Err_none;

    for (u32 i = module->numFuncImports; i < module->numFunctions; i++)
    {
        M3Function function = module->functions[i];
        _throwif("imported function", !function.wasm);
        _( ValidateFuncBody(function, module) );
    }

    _catch: return result;
}

static M3Result
ValidateData(IM3Module module)
{
    u32 num_data = module->numDataSegments;
    M3Result result = m3Err_none;

    for (u32 i = 0; i < num_data; i++)
    {
        M3DataSegment data_segment = module->dataSegments[i];
        if (data_segment.initExpr == NULL)
        {
            continue;
        }
        _throwif(m3Err_wasmMalformed, !module->memoryInfo.hasMemory)
        bytes_t i_bytes = data_segment.initExpr;
        bytes_t end = data_segment.initExpr + data_segment.initExprSize;

        i32 offset;
        _( EvaluateExpression (module, &offset, c_m3Type_i32, &i_bytes, end) );

        _throwif(m3Err_wasmMalformed, offset < 0);
    }
    
    _catch: return result;
}

M3Result
m3_ValidateModule(IM3Module module)
{
    M3Result result = m3Err_none;

    _( ValidateTables(module) );

    _( ValidateMemory(module) );

    _( ValidateGlobals(module) );

    _( ValidateStart(module) );

    _( ValidateElem(module) );

    _( ValidateDatacnt(module) );

    _( ValidateCode(module) );

    _( ValidateData(module) );
    
    _catch: return result;
}

