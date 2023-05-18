from pyubx2 import UBXReader
from serial import Serial

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


def populate_parcel_from_ubx_msg(parcel, ubx_msg):
    """With the argument ubx_msg, populate the appropriate field in the parcel dictionary"""
    message = ubx_msg.identity
    if message == 'NAV-TIMEGPS':
        parcel['gpstime'] = gps_time_from_ubx_msg(ubx_msg)
    elif message == 'NAV-HPPOSLLH':
        parcel['lat'] = ubx_msg.lat
        parcel['lon'] = ubx_msg.lon
    elif message == 'RXM-SFRBX':
        # TODO: implement correctly
        parcel['raw_gnss'] = None


def gps_listener(ubr_obj):
    """With ubr_obj as the UBX reader object, loop and listen to the incoming messages,
    sending pickled dictionaries to the ROV server"""

    parcel_dict = {'gpstime': '',
                   'lat': '',
                   'lon': '',
                   'raw_gnss': ''}

    listening = False
    while True:
        ubx_msg = ubr_obj.read()[1]
        if not listening:
            if ubx_msg.identity == 'NAV-TIMEGPS':
                # Start listening and populating parcel
                listening = True
                populate_parcel_from_ubx_msg(parcel_dict, ubx_msg)
        # We are in the listening mode
        else:
            populate_parcel_from_ubx_msg(parcel_dict, ubx_msg)
            if is_parcel_full(parcel_dict):
                # The parcel is fully populated and ready to send
                listening = False
                # TODO: pickle dict and send to ROV
                # Temporary testing code
                print(parcel_dict)
                # Empty the parcel to get ready for next listening period
                parcel_dict = emptied_parcel(parcel_dict)


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
