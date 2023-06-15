from pyubx2 import UBXReader, ubxhelpers
from serial import Serial
from time import gmtime, strftime
from datetime import datetime, timezone
import pickle


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


def utc_time_from_nav_pvt(msg):
    """Extract the UTC time from message and return as datetime object, and also return
    date and time validity and confirmation status information"""
    
    assert msg.identity == "NAV-PVT", "UBX message is not required NAV-PVT identity"
    utc_datetime = datetime(year=msg.year, 
                            month=msg.month, 
                            day=msg.day, 
                            hour=msg.hour, 
                            minute=msg.min, 
                            second=msg.second,
                            tzinfo=timezone.utc)
    # If date and time are invalid or not fully resolved, return indication of insufficiency
    # Validity and confirmation fields are 1 when true, and 0 when false (ZED-F9P Integration Manual, pg.61, R12, 3-May-2022, https://content.u-blox.com/sites/default/files/ZED-F9P_IntegrationManual_UBX-18010802.pdf)
    valid_confirmed_status = 'FULLY RESOLVED AND VALID'
    if msg.confirmedAvai == 1:
        # Date and time validity and resolution status information are available in NAV-PVT message
        if msg.validTime != 1:
            valid_confirmed_status = 'INVALID TIME'
        elif msg.validDate != 1:
            valid_confirmed_status = 'INVALID DATE'
        elif msg.fullyResolved != 1:
            valid_confirmed_status = 'NOT FULLY RESOLVED'
    else:
        valid_confirmed_status = 'DATE AND TIME VALIDITY AND RESOLUTION INFORMATION UNAVAILABLE'

    return utc_datetime, valid_confirmed_status


def raw_data_filename():
    """Return the filepath to the raw data file location"""
    return r'hw/gps_listener_sfrbx_data.ubx'


def write_to_raw_file(ubx_msg):
    """Write the raw bytes representation of the UBX message into the specified file"""
    with open(raw_data_filename(), 'ab') as rd_file:
        rd_file.write(ubx_msg)


def populate_parcel_from_ubx_msg(parcel, ubx_msg):
    """With the argument ubx_msg, populate the appropriate field in the parcel dictionary"""
    message = ubx_msg.identity
    if message == 'NAV-PVT':
        (utctime, status) = utc_time_from_nav_pvt(ubx_msg)
        parcel['utctime'], parcel['utctime-status'] = utctime, status
    elif message == 'NAV-HPPOSLLH':
        parcel['lat'] = ubx_msg.lat
        parcel['lon'] = ubx_msg.lon
    elif message == 'NAV-TIMEGPS':
        parcel['gps-iTOW'] = ubx_msg.iTOW
        parcel['gps-weeknum'] = ubx_msg.week


def gps_listener(ubr_obj):
    """With ubr_obj as the UBX reader object, loop and listen to the incoming messages,
    sending pickled dictionaries to the ROV server"""

    parcel_dict = {'gps-iTOW': '',
                   'gps-weeknum': '',
                   'utctime': '',
                   'utctime-status': '',
                   'lat': '',
                   'lon': ''}

    while True:
        (raw_bytes, msg) = ubr_obj.read()
        try:
            # Write all incoming raw bytes format of UBX messages to .ubx file
            write_to_raw_file(raw_bytes)
            # Populate parcel with general information, pickle and send
            populate_parcel_from_ubx_msg(parcel_dict, msg)
            if is_parcel_full(parcel_dict):
                # The parcel is fully populated and ready to send
                # TODO: pickle dict and send to ROV
                
                # Temporary testing code
                print(parcel_dict)
                # Empty the parcel to get ready for next listening period
                parcel_dict = emptied_parcel(parcel_dict)
                        
        # TODO: better handle unknown messages as they arrive (just ignore and pass over them?)
        except AttributeError as attr_err:
            print(attr_err)
            # print(f"The UBX message was actually this: {str(msg)} / {repr(msg)}")


if __name__ == "__main__":
    stream = Serial("/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00", 9600, timeout=5)
    ubr = UBXReader(stream)
    
    # Read latest UBX message and print the current iTOW
    # msg = ubr.read()[1]
    # print_curr_iTOW(msg)
    # print_ubx_msg_attrs(msg)
    # print_x_messages_id(ubr, 5)

    # Run GPS listener
    gps_listener(ubr)
