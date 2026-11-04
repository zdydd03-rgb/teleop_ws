"""Pure parsing; no SDK import, connection, register writes, or ROS."""
JOINT_NAMES = ('thumb_pitch', 'thumb_yaw', 'index_pitch', 'middle_pitch',
               'ring_pitch', 'little_pitch', 'thumb_roll')
WRITE_CODES = {0:'SDK acknowledged; physical completion not proven',
               1:'SDK did not obtain response before timeout; unconfirmed',
               -1:'send failed', -2:'receive failed or controller response timeout',
               -3:'response parse failed', -4:'interface unsupported on fourth-generation controller'}
READ_CODES = {0:'SDK read success; payload must still be validated',
              1:'controller false / invalid parameters or arm state (per local read-interface docstring)',
              -1:'send failed', -2:'receive failed or controller response timeout',
              -3:'response parse failed', -4:'interface unsupported on fourth-generation controller'}


def interpret_read(code, raw, *, index=None):
    result = dict(return_code=code, return_code_meaning=READ_CODES.get(code,'unknown SDK code; preserved'),
                  raw=raw, valid=False, positions=None, index=index)
    if code != 0:
        result['validation_error'] = 'nonzero return code; raw data is not a position measurement'
        return result
    if index is not None:
        if type(raw) is not int or not 0 <= raw <= 255:
            result['validation_error'] = 'single IR value is not an integer device position in0..255'
            return result
        result.update(valid=True, positions={str(index):raw})
        return result
    if not isinstance(raw,(list,tuple)) or len(raw) != 14:
        result['validation_error'] = 'seven registers require exactly14 bytes'
        return result
    if any(type(x) is not int or not -128 <= x <= 255 for x in raw):
        result['validation_error'] = 'payload contains invalid byte values'
        return result
    data = [x & 255 for x in raw]  # SDK documents int8; also accept unsigned byte representation.
    decoded = [(data[2*i]<<8) | data[2*i+1] for i in range(7)]
    if any(x > 255 for x in decoded):
        result.update(decoded_registers=decoded, validation_error='decoded uint16 register exceeds declared position domain0..255')
        return result
    result.update(valid=True, positions=decoded, byte_order='uint16 big-endian', joint_names=list(JOINT_NAMES))
    return result


def read_existing_session(adapter, *, index=None):
    """Only uses the adapter's existing arm. Exactly one Modbus READ, no fallback/retry."""
    if index not in (None,2):
        raise ValueError('This stage supports full IR0..6 or single IR2 only')
    arm = adapter._require_arm()
    if index is None:
        params = adapter.module.rm_peripheral_read_write_params_t(port=1,address=0,device=39,num=7)
        code, raw = arm.rm_read_multiple_input_registers(params)
    else:
        params = adapter.module.rm_peripheral_read_write_params_t(port=1,address=2,device=39,num=1)
        code, raw = arm.rm_read_input_registers(params)
    result = interpret_read(int(code),raw,index=index)
    result.update(operation='read_input_registers', port=1, device=39,
                  address=0 if index is None else index, num=7 if index is None else 1)
    return result
