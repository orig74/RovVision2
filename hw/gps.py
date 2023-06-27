from pyubx2 import UBXReader, ubxhelpers, UBXStreamError
from serial import Serial
from time import gmtime, strftime
from datetime import datetime, timezone
import pickle


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
    # Validity and confirmation fields are 1 when true, and 0 when false 
    # (ZED-F9P Integration Manual, pg.61, R12 3-May-2022)
    # https://content.u-blox.com/sites/default/files/ZED-F9P_IntegrationManual_UBX-18010802.pdf)
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


def populate_parcel_with_ubx_msg(parcel, ubx_msg):
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
        try:
            (raw_bytes, msg) = ubr_obj.read()
            # Write all incoming raw bytes format of UBX messages to .ubx file
            write_to_raw_file(raw_bytes)
            # Populate parcel with general information, pickle and send
            populate_parcel_with_ubx_msg(parcel_dict, msg)
            if is_parcel_full(parcel_dict):
                # The parcel is fully populated and ready to send
                # TODO: pickle dict and send to ROV
                
                # Temporary testing code
                print(parcel_dict)
                # Empty the parcel to get ready for next listening period
                parcel_dict = emptied_parcel(parcel_dict)
                        
        # Ignore unrecognised messages
        except (AttributeError, UBXStreamError) as error:
            print(error)
            pass


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
