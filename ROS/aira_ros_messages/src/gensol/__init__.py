import genmsg.msgs

try:
    from cStringIO import StringIO #Python 2.x
except ImportError:
    from io import StringIO #Python 3.x

MSG_TYPE_TO_SOL = {'byte':    'int8',
                   'char':    'uint8',
                   'bool':    'bool',
                   'uint8':   'uint8',
                   'int8':    'int8',
                   'uint16':  'uint16',
                   'int16':   'int16',
                   'uint32':  'uint32',
                   'int32':   'int32',
                   'uint64':  'uint64',
                   'int64':   'int64',
                   'float32': 'int128',
                   'float64': 'int256',
                   'string':  'string',
                   'time':     'uint',
                   'duration': 'uint'}

def msg_type_to_sol(t):
    (base_type, is_array, array_len) = genmsg.msgs.parse_type(t)
    
    sol_type = None
    if genmsg.msgs.is_builtin(base_type):
        sol_type = MSG_TYPE_TO_SOL[base_type]
    elif (len(base_type.split('/')) == 1):
        if (genmsg.msgs.is_header_type(base_type)):
            sol_type = 'std_msgs_Header'
    else:
        pkg = base_type.split('/')[0]
        msg = base_type.split('/')[1]
        sol_type = '%s_%s'%(pkg, msg)

    if (is_array):
        if (array_len is None):
            return '%s[]'%sol_type
        else:
            print('Multidimentional arrays currently does not supported by generator!')
            return None
    return sol_type
