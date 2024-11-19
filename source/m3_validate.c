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
        || ( (lim_max >= 0) && (lim_max < lim_min) )
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
        || ( (lim_max >= 0) && (lim_max < lim_min) )
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
            case 0x41:  // i32.const
            {
                init_expr_type = c_m3Type_i32;
                break;
            }
            case 0x42:  // i64.const
            {
                init_expr_type = c_m3Type_i64;
                break;
            }
            case 0x43:  // f32.const
            {
                init_expr_type = c_m3Type_f32;
                break;
            }
            case 0x44:  // f64.const
            {
                init_expr_type = c_m3Type_f64;
                break;
            }
            case 0x23: // global.get
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
    _throwif(m3Err_wasmMalformed, start_idx >= module->numFunctions);

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
    _throwif(m3Err_wasmMalformed, (op != 0x41)); // i32.const

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

static M3Result
ValidateCode(IM3Module module)
{
    return m3Err_none;
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
        _throwif(m3Err_wasmMalformed, (op != 0x41)); // i32.const

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

