#include "wasm3.h"
#include "m3_exception.h"
#include "m3_compile.h"
#include "m3_env.h"


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


typedef struct _frame_ll {
    u16 numRets;
    u16 numArgs;
    u8* types;
    struct _frame_ll* next;
} FrameLL;

typedef struct {
    bytes_t expression;
    bytes_t end;
    FrameLL* frames;
} WorkItem;

typedef struct _WorkNode {
    WorkItem item;
    struct _WorkNode* next;
} WorkNode;

typedef struct {
    WorkNode* head; // must not be pointed at by other nodes if not NULL,
                     //   must not be NULL if tail is not NULL
                     //   must point to NULL iff tail is NULL

    WorkNode* tail;  // must point to NULL if not NULL
} WorkChain;

static void
PushTail(WorkChain* chain, WorkItem item)
{
    WorkNode* next = m3_Malloc(sizeof(WorkNode));
    next->item = item;
    next->next = NULL;
    if (!chain->tail)
    {
        if (!chain->head)
        {
            chain->head = next;  // [NULL, NULL] -> [head->NULL, NULL]
        }
        else
        {
            chain->tail = next;
            chain->head->next = next; // [head->NULL, NULL] -> [head->tail, tail]
        }
    }
    else
    {
        chain->tail->next = next;
        chain->tail = next;
    }
}

static bool
PopHead(WorkChain* chain, WorkItem* result)
{
    if (!chain->head)
    {
        return 0;
    }
    WorkNode* popped = chain->head;
    *result = popped->item;
    if (!chain->tail)
    {
        chain->head = NULL;
    }
    else if (chain->head->next == chain->tail)
    {
        chain->head = chain->tail;
        chain->tail = NULL;
    }
    else
    {
        chain->head = popped->next;
    }
    m3_Free(popped);
    return 1;
}

// transfer control of item
//
static M3Result
ValidateExpression(WorkItem item, WorkChain* chain, u8* type_stack)
{
    M3Result result = m3Err_none;

    //  iterate through opcodes, pushing/popping types on the stack
    //  unconditional control transfer ops: consult frame list
    //  push block ops to work chain
    //  free item.frames when done

    _catch: return result;
}

static M3Result
SkipBlockBody(bytes_t * io_bytes, cbytes_t i_end)
{
    M3Result result = m3Err_none;

    
    
    _catch: return result;
}

static M3Result
ValidateFuncBody(
    bytes_t wasm,
    bytes_t wasm_end,
    M3FuncType type,
    IM3Module module)
{
    M3Result result = m3Err_none;
    u8* type_stack = m3_Malloc(d_m3MaxSaneTypeStackSize);

    _throwif(
        m3Err_wasmMalformed,
        (type.numRets + type.numArgs > d_m3MaxSaneLocalCount)
    );
    

    u8 local_types[d_m3MaxSaneLocalCount];
    u32 j = 0;
    for (u32 i = type.numRets; i < type.numRets + type.numArgs; i++)
    {
        local_types[j] = type.types[i];
        j++;
    }
    u32 num_local_blocks;
    _( ReadLEB_u32(&num_local_blocks, &wasm, wasm_end) );

    for (u32 i = 0; i < num_local_blocks; i++)
    {
        u32 num_locals;
        i8 waType;
        u8 type;
        _( ReadLEB_u32(&num_locals, &wasm, wasm_end) );
        _( ReadLEB_i7(&waType, &wasm, wasm_end) );
        _( NormalizeType (&type, waType) );
        for (u32 k = 0; k < num_locals; k++)
        {
            _throwif(m3Err_wasmMalformed, (j > d_m3MaxSaneLocalCount));
            local_types[j] = type;
            j++;
        }
    }
    FrameLL* init_frame = m3_Malloc(sizeof(*init_frame));
    init_frame->numRets = type.numRets;
    init_frame->numArgs = type.numArgs;
    init_frame->types = type.types;
    init_frame->next = NULL;

    WorkNode* init_node = m3_Malloc(sizeof(*init_node));
    WorkItem init_item = {wasm, wasm_end, init_frame};
    init_node->item = init_item;
    init_node->next = NULL;

    WorkChain chain = {init_node, NULL};
    
    while (chain.head)
    {
        WorkItem item;
        PopHead(&chain, &item);
        _( ValidateExpression(item, &chain, type_stack) );  //  item transfer
    }

    _catch: {
        //  the workchain is either empty or the validation failed
        //  and the memory is going to be reclaimed soon
        //
        m3_Free(type_stack);
        return result;
    }
}

static M3Result
ValidateCode(IM3Module module)
{
    M3Result result = m3Err_none;

    for (u32 i = module->numFuncImports; i < module->numFunctions; i++)
    {
        M3Function function = module->functions[i];
        M3FuncType type = *function.funcType;
        _( ValidateFuncBody(function.wasm, function.wasmEnd, type, module) );
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

