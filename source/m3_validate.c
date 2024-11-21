#include "wasm3.h"
#include "m3_exception.h"
#include "m3_compile.h"
#include "m3_env.h"
#include "m3_bind.h"

static M3Result
ValidateFunctions(IM3Module module)
{
    return m3Err_none;  //  parser already checked functypes
}

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
    u8 label;
    _( Read_u8(&label, &i_bytes, end) );
    _throwif(m3Err_wasmMalformed, label);
    
    u8 op;
    _( Read_u8(&op, &i_bytes, end) );
    _throwif(m3Err_wasmMalformed, (op != c_waOp_i32_const));

    i32 _discard;
    _( ReadLEB_i32(&_discard, &i_bytes, end) );

    u32 num_idxes;
    _( ReadLEB_u32(&num_idxes, &i_bytes, end) );

    while (num_idxes--)
    {
        u32 func_idx;
        _( ReadLEB_u32(&func_idx, &i_bytes, end) );

        _throwif(m3Err_wasmMalformed, func_idx >= module->numFunctions);
    }

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

// only defined for ops with non-NULL signatures
static M3Result
SkipImmediateArgs(m3opcode_t op,
  i8 numArgsImmediate,
  bytes_t *io_bytes,
  cbytes_t end)
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
            default:
            {
                _throw(m3Err_wasmMalformed);
            }
        }
    }

    _catch: return result;
}

static M3Result
SkipPastEndElse(bool* is_else, bytes_t *io_bytes, cbytes_t end)
{
    M3Result result = m3Err_none;  // TODO

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
    {
        IM3FuncType type = function.funcType;
        u32 j = 0;
        for (u32 i = type->numRets; i < type->numRets + type->numArgs; i++)
        {
            local_types[j] = type->types[i];
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
                _throwif(m3Err_wasmMalformed, (j > d_m3MaxSaneLocalCount));
                local_types[j] = type;
                j++;
            }
        }
        total_locals = j;
    }

    u8 type_stack[d_m3MaxSaneTypeStackSize];
    Frame frames[d_m3MaxSaneFrameDepth];

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
        if (info->typeSignature)
        {
            ccstr_t signature = info->typeSignature;
            {
                u8 i_arg = info->numArgs;
                while (i_arg--)
                {
                    _throwif(m3Err_wasmMalformed, (type_i == 0));

                    u8 type_id = ConvertTypeCharToTypeId(signature[i_arg]);
                    _throwif(
                        m3Err_wasmMalformed,
                        (type_stack[--type_i] != type_id)
                    );
                }
            }
            u8 num_tot = info->numArgs + info->numRets;
            for (u8 i_ret = info->numArgs; i_ret < num_tot; i_ret++ )
            {
                _throwif(
                    m3Err_wasmMalformed,
                    (type_i >= d_m3MaxSaneTypeStackSize)
                );
                u8 type = ConvertTypeCharToTypeId(signature[i_ret]);
                type_stack[type_i++] = type;
            }
            
            i8 imm_args = info->numArgsImmediate;
            if (imm_args != 0)
            {
                _(SkipImmediateArgs(op, imm_args, &io_bytes, end));
            }
        }
        else
        {
            switch (op)
            {
                // "dynamic" types
                case c_waOp_nop:
                {
                    continue;
                }
                case c_waOp_branchIf:
                {
                    _throwif(
                        m3Err_wasmMalformed,
                        ( (type_i == 0)
                          || (type_stack[--type_i] != c_m3Type_i32)
                        )
                    );
                    u32 depth;
                    _( ReadLEB_u32(&depth, &io_bytes, end) );
                    
                    _throwif(m3Err_wasmMalformed, (depth >= frame_i));

                    Frame frame = frames[frame_i-depth];

                    if (frame.kind == bk_loop)
                    {
                        IM3FuncType blocktype = frame.blocktype;
                        u16 numArgs = blocktype->numArgs;
                        u16 numRets = blocktype->numRets;
                        u16 arg_i = numArgs + numRets;
                        u32 type_i_temp = type_i;
                        u8 *types = blocktype->types;

                        while ((arg_i--) > numRets)
                        {
                            _throwif(m3Err_wasmMalformed, type_i_temp == 0);
                            _throwif(
                                m3Err_wasmMalformed,
                                type_stack[--type_i_temp] != types[arg_i]
                            );
                        }
                    }
                    else
                    {
                        IM3FuncType blocktype = frame.blocktype;
                        u16 ret_i = blocktype->numRets;
                        u32 type_i_temp = type_i;
                        u8 *types = blocktype->types;

                        while ((ret_i--) > 0)
                        {
                            _throwif(m3Err_wasmMalformed, type_i_temp == 0);
                            _throwif(
                                m3Err_wasmMalformed,
                                type_stack[--type_i_temp] != types[ret_i]
                            );
                        }
                    }

                    continue;
                }
                case c_waOp_call:
                {
                    u32 func_idx;
                    _( ReadLEB_u32(&func_idx, &io_bytes, end) );
                    _throwif(
                        m3Err_wasmMalformed,
                        (func_idx >= module->numFunctions)
                    );

                    IM3FuncType functype = module->functions[func_idx].funcType;
                    u16 numArgs = functype->numArgs;
                    u16 numRets = functype->numRets;
                    u8 *types = functype->types;
                    u16 arg_i = numArgs + numRets;

                    while ((arg_i--) > numRets)
                    {
                        _throwif(m3Err_wasmMalformed, type_i == 0);
                        _throwif(
                            m3Err_wasmMalformed,
                            (type_stack[--type_i] != types[arg_i])
                        );
                    }
                    for (u16 ret_i = 0; ret_i < numRets; ret_i++)
                    {
                        type_stack[type_i++] = types[ret_i];
                    }
                    
                    continue;
                }
                case c_waOp_call_indirect:
                {
                    u32 type_idx, table_idx;
                    _( ReadLEB_u32(&type_idx, &io_bytes, end) );
                    _( ReadLEB_u32(&table_idx, &io_bytes, end) );

                    _throwif(m3Err_wasmMalformed, table_idx != 0);  // only one table supported
                    _throwif(m3Err_wasmMalformed, type_i == 0);
                    _throwif(
                        m3Err_wasmMalformed,
                        (type_stack[--type_i] != c_m3Type_i32)
                    );
                    _throwif(
                        m3Err_wasmMalformed,
                        (type_idx >= module->numFuncTypes)
                    );
                    IM3FuncType functype = module->funcTypes[type_idx];
                    u16 numArgs = functype->numArgs;
                    u16 numRets = functype->numRets;
                    u8 *types = functype->types;
                    u16 arg_i = numArgs + numRets;

                    while ((arg_i--) > numRets)
                    {
                        _throwif(m3Err_wasmMalformed, type_i == 0);
                        _throwif(
                            m3Err_wasmMalformed,
                            (type_stack[--type_i] != types[arg_i])
                        );
                    }
                    for (u16 ret_i = 0; ret_i < numRets; ret_i++)
                    {
                        type_stack[type_i++] = types[ret_i];
                    }
                    
                    continue;
                }
                case c_waOp_drop:
                {
                    _throwif(m3Err_wasmMalformed, (type_i--) == 0);
                    continue;
                }
                case c_waOp_select:
                {
                    _throwif(m3Err_wasmMalformed, type_i < 3);
                    _throwif(
                        m3Err_wasmMalformed,
                        (type_stack[--type_i] != c_m3Type_i32)
                    );
                    u8 t1 = type_stack[--type_i];
                    u8 t2 = type_stack[--type_i];

                    _throwif(m3Err_wasmMalformed, (t1 != t2));

                    continue;
                }
                case c_waOp_getLocal:
                {
                    u32 local_idx;
                    _( ReadLEB_u32(&local_idx, &io_bytes, end) );

                    _throwif(m3Err_wasmMalformed, local_idx >= total_locals);

                    type_stack[type_i++] = local_types[local_idx];

                    continue;
                }
                case c_waOp_setLocal:
                {
                    u32 local_idx;
                    _( ReadLEB_u32(&local_idx, &io_bytes, end) );

                    _throwif(m3Err_wasmMalformed, local_idx >= total_locals);

                    _throwif(m3Err_wasmMalformed, type_i == 0);

                    _throwif(
                        m3Err_wasmMalformed,
                        (type_stack[--type_i] != local_types[local_idx])
                    );

                    continue;
                }
                case c_waOp_teeLocal:
                {
                    u32 local_idx;
                    _( ReadLEB_u32(&local_idx, &io_bytes, end) );

                    _throwif(m3Err_wasmMalformed, local_idx >= total_locals);

                    _throwif(m3Err_wasmMalformed, type_i == 0);

                    _throwif(
                        m3Err_wasmMalformed,
                        (type_stack[type_i-1] != local_types[local_idx])
                    );

                    continue;
                }
                case c_waOp_getGlobal:
                {
                    u32 global_idx;
                    _( ReadLEB_u32(&global_idx, &io_bytes, end) );

                    _throwif(
                        m3Err_wasmMalformed,
                        (global_idx >= module->numGlobals)
                    );

                    M3Global global = module->globals[global_idx];
                    type_stack[type_i++] = global.type;

                    continue;
                }
                case c_waOp_setGlobal:
                {
                    u32 global_idx;
                    _( ReadLEB_u32(&global_idx, &io_bytes, end) );

                    _throwif(
                        m3Err_wasmMalformed,
                        (global_idx >= module->numGlobals)
                    );

                    M3Global global = module->globals[global_idx];

                    _throwif(m3Err_wasmMalformed, !global.isMutable);

                    _throwif(m3Err_wasmMalformed, type_i == 0);

                    _throwif(
                        m3Err_wasmMalformed,
                        (type_stack[--type_i] != global.type)
                    );

                    continue;
                }

                // control: leave frame
                case c_waOp_unreachable:
                {
                    // go to the end of current block
                    bool is_else;
                    _( SkipPastEndElse(&is_else, &io_bytes, end) );

                    if (is_else) // switch the frame
                    {
                        u32 now_i = frame_i - 1;
                        bool frame_is_if = (bk_if == frames[now_i].kind);
                        bool frame_true_branch = frames[now_i].is_true_branch;
                        _throwif(
                            m3Err_wasmMalformed,
                            !(frame_is_if && frame_true_branch)
                        );
                        frames[now_i].is_true_branch = 0;
                        type_i = frames[now_i].base_i;

                        continue;
                    }
                    else
                    {
                        Frame popped = frames[--frame_i];
                        type_i = popped.base_i;
                        
                        u32 numRets = popped.blocktype->numRets;
                        u8 *types = popped.blocktype->types;
                        for (u32 i = 0; i < numRets; i++)
                        {
                            type_stack[type_i++] = types[i];
                        }

                        continue;
                    }
                }
                case c_waOp_else:
                {
                    Frame *now = &frames[frame_i-1];
                    bool frame_is_if = (bk_if == now->kind);
                    bool frame_true_branch = now->is_true_branch;
                    _throwif(
                        m3Err_wasmMalformed,
                        !(frame_is_if && frame_true_branch)
                    );
                    _throwif(m3Err_wasmMalformed, (type_i < now->base_i));
                    u16 numRets = now->blocktype->numRets;
                    u8* types = now->blocktype->types;
                    u16 ret_i = numRets;
                    while (ret_i--)
                    {
                        _throwif(
                            m3Err_wasmMalformed,
                            types[ret_i] != type_stack[--type_i]
                        );
                    }
                    _throwif(m3Err_wasmMalformed, (type_i != now->base_i));
                    now->is_true_branch = 0;

                    continue;
                }
                case c_waOp_end:
                {
                    Frame popped = frames[--frame_i];
                    _throwif(m3Err_wasmMalformed, (type_i < popped.base_i));
                    u16 numRets = popped.blocktype->numRets;
                    u8* types = popped.blocktype->types;
                    u16 ret_i = numRets;
                    while (ret_i--)
                    {
                        _throwif(
                            m3Err_wasmMalformed,
                            types[ret_i] != type_stack[--type_i]
                        );
                    }
                    _throwif(m3Err_wasmMalformed, (type_i != popped.base_i));

                    continue;
                }
                case c_waOp_branch:
                {
                    u32 depth;
                    _( ReadLEB_u32(&depth, &io_bytes, end) );

                    _throwif(m3Err_wasmMalformed, (depth >= frame_i));

                    Frame frame = frames[frame_i-depth];

                    if (frame.kind == bk_loop)
                    {
                        IM3FuncType blocktype = frame.blocktype;
                        u16 numArgs = blocktype->numArgs;
                        u16 numRets = blocktype->numRets;
                        u16 arg_i = numArgs + numRets;
                        u32 type_i_temp = type_i;
                        u8 *types = blocktype->types;

                        while ((arg_i--) > numRets)
                        {
                            _throwif(m3Err_wasmMalformed, type_i_temp == 0);
                            _throwif(
                                m3Err_wasmMalformed,
                                type_stack[--type_i_temp] != types[arg_i]
                            );
                        }
                    }
                    else
                    {
                        IM3FuncType blocktype = frame.blocktype;
                        u16 ret_i = blocktype->numRets;
                        u32 type_i_temp = type_i;
                        u8 *types = blocktype->types;

                        while ((ret_i--) > 0)
                        {
                            _throwif(m3Err_wasmMalformed, type_i_temp == 0);
                            _throwif(
                                m3Err_wasmMalformed,
                                type_stack[--type_i_temp] != types[ret_i]
                            );
                        }
                    }

                    // go to the end of the current block
                    bool is_else;
                    _( SkipPastEndElse(&is_else, &io_bytes, end) );

                    if (is_else) // switch the frame
                    {
                        u32 now_i = frame_i - 1;
                        bool frame_is_if = (bk_if == frames[now_i].kind);
                        bool frame_true_branch = frames[now_i].is_true_branch;
                        _throwif(
                            m3Err_wasmMalformed,
                            !(frame_is_if && frame_true_branch)
                        );
                        frames[now_i].is_true_branch = 0;
                        type_i = frames[now_i].base_i;
                        continue;
                    }
                    else
                    {
                        Frame popped = frames[--frame_i];
                        type_i = popped.base_i;
                        
                        u32 numRets = popped.blocktype->numRets;
                        u8 *types = popped.blocktype->types;
                        for (u32 i = 0; i < numRets; i++)
                        {
                            type_stack[type_i++] = types[i];
                        }
                        continue;
                    }
                }
                case c_waOp_branchTable:
                {
                    u32 n_depths;
                    _( ReadLEB_u32(&(n_depths), &io_bytes, end) );
                    n_depths++;

                    for (u32 i = 0; i < n_depths; i++)
                    {
                        u32 depth;
                        _( ReadLEB_u32(&depth, &io_bytes, end) );
                        _throwif(m3Err_wasmMalformed, (depth >= frame_i));

                        Frame frame = frames[frame_i-depth];

                        if (frame.kind == bk_loop)
                        {
                            IM3FuncType blocktype = frame.blocktype;
                            u16 numArgs = blocktype->numArgs;
                            u16 numRets = blocktype->numRets;
                            u16 arg_i = numArgs + numRets;
                            u32 type_i_temp = type_i;
                            u8 *types = blocktype->types;

                            while ((arg_i--) > numRets)
                            {
                                _throwif(m3Err_wasmMalformed, type_i_temp == 0);
                                _throwif(
                                    m3Err_wasmMalformed,
                                    type_stack[--type_i_temp] != types[arg_i]
                                );
                            }
                        }
                        else
                        {
                            IM3FuncType blocktype = frame.blocktype;
                            u16 ret_i = blocktype->numRets;
                            u32 type_i_temp = type_i;
                            u8 *types = blocktype->types;

                            while ((ret_i--) > 0)
                            {
                                _throwif(m3Err_wasmMalformed, type_i_temp == 0);
                                _throwif(
                                    m3Err_wasmMalformed,
                                    type_stack[--type_i_temp] != types[ret_i]
                                );
                            }
                        }
                    }

                    // go to the end of the current block
                    bool is_else;
                    _( SkipPastEndElse(&is_else, &io_bytes, end) );

                    if (is_else) // switch the frame
                    {
                        u32 now_i = frame_i - 1;
                        bool frame_is_if = (bk_if == frames[now_i].kind);
                        bool frame_true_branch = frames[now_i].is_true_branch;
                        _throwif(
                            m3Err_wasmMalformed,
                            !(frame_is_if && frame_true_branch)
                        );
                        frames[now_i].is_true_branch = 0;
                        type_i = frames[now_i].base_i;

                        continue;
                    }
                    else
                    {
                        Frame popped = frames[--frame_i];
                        type_i = popped.base_i;
                        
                        u32 numRets = popped.blocktype->numRets;
                        u8 *types = popped.blocktype->types;
                        for (u32 i = 0; i < numRets; i++)
                        {
                            type_stack[type_i++] = types[i];
                        }

                        continue;
                    }
                }
                case c_waOp_return:
                {
                    Frame frame = frames[0];
                    IM3FuncType blocktype = frame.blocktype;
                    u16 ret_i = blocktype->numRets;
                    u32 type_i_temp = type_i;
                    u8 *types = blocktype->types;

                    while ((ret_i--) > 0)
                    {
                        _throwif(m3Err_wasmMalformed, type_i_temp == 0);
                        _throwif(
                            m3Err_wasmMalformed,
                            type_stack[--type_i_temp] != types[ret_i]
                        );
                    }

                    // go to the end of the current block
                    bool is_else;
                    _( SkipPastEndElse(&is_else, &io_bytes, end) );

                    if (is_else) // switch the frame
                    {
                        u32 now_i = frame_i - 1;
                        bool frame_is_if = (bk_if == frames[now_i].kind);
                        bool frame_true_branch = frames[now_i].is_true_branch;
                        _throwif(
                            m3Err_wasmMalformed,
                            !(frame_is_if && frame_true_branch)
                        );
                        frames[now_i].is_true_branch = 0;
                        type_i = frames[now_i].base_i;
                        continue;
                    }
                    else
                    {
                        Frame popped = frames[--frame_i];
                        type_i = popped.base_i;
                        
                        u32 numRets = popped.blocktype->numRets;
                        u8 *types = popped.blocktype->types;
                        for (u32 i = 0; i < numRets; i++)
                        {
                            type_stack[type_i++] = types[i];
                        }
                        continue;
                    }
                }
                

                // control: enter frame
                case c_waOp_block:
                {
                    IM3FuncType blocktype;
                    _( ParseBlockType(&blocktype, &io_bytes, end, module) );

                    u16 numArgs = blocktype->numArgs;
                    u16 numRets = blocktype->numRets;
                    u8 *types = blocktype->types;
                    u16 arg_i = numArgs + numRets;
                    while ((arg_i--) > numRets) 
                    {
                        _throwif(m3Err_wasmMalformed, type_i == 0);
                        _throwif(
                            m3Err_wasmMalformed,
                            type_stack[--type_i] != types[arg_i]
                        );
                    }

                    Frame new = {bk_block, 0, type_i, blocktype};
                    frames[frame_i++] = new;
                }
                case c_waOp_loop:
                {
                    IM3FuncType blocktype;
                    _( ParseBlockType(&blocktype, &io_bytes, end, module) );

                    u16 numArgs = blocktype->numArgs;
                    u16 numRets = blocktype->numRets;
                    u8 *types = blocktype->types;
                    u16 arg_i = numArgs + numRets;
                    while ((arg_i--) > numRets) 
                    {
                        _throwif(m3Err_wasmMalformed, type_i == 0);
                        _throwif(
                            m3Err_wasmMalformed,
                            type_stack[--type_i] != types[arg_i]
                        );
                    }

                    Frame new = {bk_loop, 0, type_i, blocktype};
                    frames[frame_i++] = new;
                }
                case c_waOp_if:
                {
                    IM3FuncType blocktype;
                    _( ParseBlockType(&blocktype, &io_bytes, end, module) );

                    u16 numArgs = blocktype->numArgs;
                    u16 numRets = blocktype->numRets;
                    u8 *types = blocktype->types;
                    u16 arg_i = numArgs + numRets;
                    while ((arg_i--) > numRets) 
                    {
                        _throwif(m3Err_wasmMalformed, type_i == 0);
                        _throwif(
                            m3Err_wasmMalformed,
                            type_stack[--type_i] != types[arg_i]
                        );
                    }

                    Frame new = {bk_if, 1, type_i, blocktype};
                    frames[frame_i++] = new;
                }

                // weird: not covered by the formal spec
                case c_waOp_return_call:
                case c_waOp_return_call_indirect:
                {
                    _throw(m3Err_wasmMalformed);
                }
                default:
                {
                    _throw(m3Err_wasmMalformed);
                }
            }
        }
    }

    _catch: return result;
}


static M3Result
ValidateCode(IM3Module module)
{
    M3Result result = m3Err_none;

    for (u32 i = module->numFuncImports; i < module->numFunctions; i++)
    {
        M3Function function = module->functions[i];
        M3FuncType type = *function.funcType;
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
        bytes_t i_bytes = data_segment.initExpr;
        bytes_t end = data_segment.initExpr + data_segment.initExprSize;

        u8 op;
        _( Read_u8(&op, &i_bytes, end) );
        _throwif(m3Err_wasmMalformed, (op != c_waOp_i32_const));

        i32 _discard;
        _( ReadLEB_i32(&_discard, &i_bytes, end) );
    }
    
    _catch: return result;
}


M3Result
m3_ValidateModule(IM3Module module)
{
    M3Result result = m3Err_none;
    _try
    {
        //  imports and exports are validated implicitly?
        //        
        _( ValidateFunctions(module) );
        _( ValidateTables(module) );
        _( ValidateMemory(module) );
        _( ValidateGlobals(module) );
        _( ValidateStart(module) );
        _( ValidateElem(module) );
        _( ValidateDatacnt(module) );
        _( ValidateCode(module) );
        _( ValidateData(module) );
    }
    _catch: {}
    return result;
}

