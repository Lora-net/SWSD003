import datetime
from serial import Serial


class GnssDateLocBuilder:
    EPOCH_TIME = datetime.datetime(1980, 1, 6)

    def __init__(self):
        pass

    @staticmethod
    def get_delay_due_to_leap_second():
        """Return the delay due to leap second at the time of writing this function..."""
        return datetime.timedelta(seconds=18)

    @staticmethod
    def date_to_gps_format(date):
        delta_since_epoch = date - GnssDateLocBuilder.EPOCH_TIME
        return int(delta_since_epoch.total_seconds())

    @staticmethod
    def get_now_gps():
        today = (
            datetime.datetime.utcnow()
            + GnssDateLocBuilder.get_delay_due_to_leap_second()
        )
        gps_seconds = GnssDateLocBuilder.date_to_gps_format(today)
        return gps_seconds


if __name__ == "__main__":
    from argparse import ArgumentParser

    parser = ArgumentParser()
    parser.add_argument(
        "port",
        help="Virtual COM port where the device is connected",
    )
    args = parser.parse_args()
    with Serial(args.port, 921600, timeout=1, write_timeout=0) as ser:
        date_gps = GnssDateLocBuilder.get_now_gps()
        ser.write(f"{date_gps}\n".encode("UTF-8"))
        while True:
            line = ser.readline().decode("UTF-8")
            if line:
                print(line, end="")
