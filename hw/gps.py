from pyubx2 import UBXReader
from serial import Serial
from time import gmtime, strftime

UBX_MSG_IDs = {'NAV-HPPOSLLH': {'class': b'\x01', 'id': b'\x014'}}


def is_ubx_msg_this_type(msg_name, ubr_msg):
    """Check if the message represented by ubr_obj is of the type specified by msg_name"""

    return ubr_msg.identity == msg_name


def print_curr_iTOW(ubr_msg):
    """Prints the current integer time of week from the UBX GPS message"""

    if ubr_msg.identity == 'NAV-HPPOSLLH':
        print(f"iTOW: {ubr_msg.iTOW / 1e3} sec ")
    else:
        print(f"Message type was not NAV-HPPOSLLH")


def print_ubx_msg_attrs(ubr_msg):
    """Print the message ID, message class, and message mode"""

    print(f"Message ID: {ubr_msg.msg_id}\nMessage class: {ubr_msg.msg_cls}\nMessage mode: {ubr_msg.msgmode}")


def print_x_messages_id(ubr_obj, num_msgs):
    """Prints the ID of the incoming messages to the console, using
     the specified number of messages"""

    for _ in range(num_msgs):
        # Read incoming message
        parsed_data = ubr_obj.read()[1]
        # Obfuscated, but fun, and highly abstracted way of accessing the
        # message and UBX IDs of the incoming messages
        props = ['msg_id']
        local_vars = locals()
        for prop in props:
            # Access property from parsed data, convert to hex, and store to
            # local variables

            # NOTE: Python automtically convert bytes data into ASCII at standard
            # output where applicable, so we convert to hex to preserve byte values
            local_vars[prop] = getattr(parsed_data, prop).hex()

        prop_texts = [f"{prop}: 0x{local_vars[prop]}" for prop in props]
        props_summary = ' '.join(prop_texts)
        print(props_summary)


def is_parcel_full(parcel):
    """Return true if all the values for the keys are not empty"""
    return all([value != '' for value in parcel.values()])


def emptied_parcel(parcel):
    """Set values for all keys to be zero for the parcel dictionary"""
    return dict.fromkeys(parcel, '')


def gps_time_from_ubx_msg(ubx_msg):
    """Extract the GPS time from the message"""
    #TODO: implement correctly
    return ubx_msg.iTOW


def raw_data_filename():
    """Return the filepath to the raw data file location"""
    return r'gps_listener_sfrbx_data.ubx'


def write_to_raw_file(ubx_msg):
    """Write the SFRBX raw data to the raw data file"""
    assert ubx_msg.identity == "NAV-SFRBX", "UBX message is not the required NAV-SFRBX identity"
    
    # Serialise UBX message into bytes for file writing
    msg_bytes = ubx_msg.serialise()

    with open(raw_data_filename(), 'w') as rd_file:
        rd_file.write(msg_bytes)



def populate_parcel_from_ubx_msg(parcel, ubx_msg):
    """With the argument ubx_msg, populate the appropriate field in the parcel dictionary"""
    message = ubx_msg.identity
    if message == 'NAV-TIMEGPS':
        parcel['gpstime'] = gps_time_from_ubx_msg(ubx_msg)
    elif message == 'NAV-HPPOSLLH':
        parcel['lat'] = ubx_msg.lat
        parcel['lon'] = ubx_msg.lon


def gps_listener(ubr_obj):
    """With ubr_obj as the UBX reader object, loop and listen to the incoming messages,
    sending pickled dictionaries to the ROV server"""

    parcel_dict = {'gpstime': '',
                   'lat': '',
                   'lon': ''}

    while True:
        ubx_msg = ubr_obj.read()[1]
        try:
            if ubx_msg.identity == 'NAV-SFRBX':
                # Write raw GNSS SFRBX data directly to specified file
                write_to_raw_file(ubx_msg)
            else:
                # It is a general message
                populate_parcel_from_ubx_msg(parcel_dict, ubx_msg)
                if is_parcel_full(parcel_dict):
                    # The parcel is fully populated and ready to send
                    # TODO: pickle dict and send to ROV
                    # Temporary testing code
                    print(parcel_dict)
                    # Empty the parcel to get ready for next listening period
                    parcel_dict = emptied_parcel(parcel_dict)
                    
        except AttributeError as attr_err:
                print(attr_err)
                print(f"The UBX message was actually this: {str(ubx_msg)} / {repr(ubx_msg)}")


if __name__ == "__main__":
    stream = Serial("/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00", 9600, timeout=5)
    ubr = UBXReader(stream)
    msg = ubr.read()[1]

    # Read latest UBX message and print the current iTOW
    # print_curr_iTOW(msg)
    # print_ubx_msg_attrs(msg)
    # print_x_messages_id(ubr, 5)

    # Run GPS reader
    gps_listener(ubr)
