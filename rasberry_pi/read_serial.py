import serial
import pandas as pd
import requests
import json
import time
from pyubx2 import UBXReader, UBXMessage, SET
import warnings
import numpy as np

warnings.simplefilter(action='ignore', category=FutureWarning)

# URL API strežnika
API_URL = "http://127.0.0.1:5000/test"  #"http://192.168.1.123:5000/api/test"

# Funkcija za vklop NAV-SVINFO, NAV-CLOCK, NAV-POSLLH in NAV-TIMEUTC
def configure_receiver(ser):
    """
    Nastavi GNSS sprejemnik tako, da omogoči le sporočila MON-RF in NAV-SAT,
    ostala izklopi.
    """
    messages = [
        # Omogoči samo MON-RF in NAV-SAT
        UBXMessage(0x06, 0x01, SET, msgClass=0x0A, msgID=0x38, rateUART1=1, rateUSB=1),  # MON-RF
        UBXMessage(0x06, 0x01, SET, msgClass=0x01, msgID=0x35, rateUART1=1, rateUSB=1),  # NAV-SAT
        UBXMessage(0x06, 0x01, SET, msgClass=0x01, msgID=0x21, rateUART1=1, rateUSB=1),  # NAV-TIMEUTC

        # Izklopi NAV-CLOCK
        UBXMessage(0x06, 0x01, SET, msgClass=0x01, msgID=0x22, rateUART1=0, rateUSB=0),
        # Izklopi RXM-MEASX
        UBXMessage(0x06, 0x01, SET, msgClass=0x02, msgID=0x14, rateUART1=0, rateUSB=0),
        # Izklopi MON-HW
        UBXMessage(0x06, 0x01, SET, msgClass=0x0A, msgID=0x09, rateUART1=0, rateUSB=0),
        # Izklopi MON-HW2
        UBXMessage(0x06, 0x01, SET, msgClass=0x0A, msgID=0x0B, rateUART1=0, rateUSB=0),
        # Izklopi MON-SPAN
        UBXMessage(0x06, 0x01, SET, msgClass=0x0A, msgID=0x31, rateUART1=0, rateUSB=0),
    ]

    for msg in messages:
        ser.write(msg.serialize())
        print(f"Poslano sporočilo: {msg.identity}", flush=True)

def set_baudrate_ublox(ser):
    """
    Pošlje UBX-CFG-PRT sporočilo za nastavitev baudrate na 115200 za UART1.
    """
    msg = UBXMessage(
        0x06, 0x00, SET,
        portID=1,
        reserved0=0,
        txReady=0,
        mode=0x08D0,          # 8N1
        baudRate=115200,
        inProtoMask=0x0007,   # UBX + NMEA + RTCM
        outProtoMask=0x0001,  # UBX
        flags=0,
        reserved5=0
    )
    ser.write(msg.serialize())
    print("Poslano: nastavitev baudrate na 115200")

def extract_nav_sat(ubx_msg):
    """
    Iz UBX-NAV-SAT sporočila vrne sat podatke.
    """
    GPS_avg = 0
    GPS_count = 0
    GALILEO_avg = 0
    GALILEO_count = 0
    GLONASS_avg = 0
    GLONASS_count = 0
    BEIDOU_avg = 0
    BEIDOU_count = 0
    data = {
        'iTOW': ubx_msg.iTOW,
        'sat_version': ubx_msg.version,
        'sat_numSvs': ubx_msg.numSvs
    }

    for i in range(1, ubx_msg.numSvs + 1):
        system = getattr(ubx_msg, f'gnssId_{i:02}')
        sat_num = getattr(ubx_msg, f'svId_{i:02}')
        system_ids = {0: "GPS", 1: "SBAS", 2: "GALILEO", 3: "BEIDOU", 5:"QZSS", 6:"GLONASS"}
        system_id = system_ids.get(system, "UNKNOWN")
        cn = getattr(ubx_msg, f'cno_{i:02}')
        used = getattr(ubx_msg, f'svUsed_{i:02}')
        elev = getattr(ubx_msg, f'elev_{i:02}')
        if system_id == "GPS":
            GPS_avg += cn*used*np.sin(np.radians(elev))
            GPS_count += used*np.sin(np.radians(elev))
        elif system_id == "GLONASS":
            GLONASS_avg += cn*used*np.sin(np.radians(elev))
            GLONASS_count += used*np.sin(np.radians(elev))
        elif system_id == "GALILEO":
            GALILEO_avg += cn*used*np.sin(np.radians(elev))
            GALILEO_count += used*np.sin(np.radians(elev))
        elif system_id == "BEIDOU":
            BEIDOU_avg += cn*used*np.sin(np.radians(elev))
            BEIDOU_count += used*np.sin(np.radians(elev))

        data.update({
            f'sat_cno_{system_id}_{sat_num:02}': cn,
            f'sat_elev_{system_id}_{sat_num:02}': elev,
            f'sat_svUsed_{system_id}_{sat_num:02}': used,
        })
    data.update({
        'GPS_avg': GPS_avg/GPS_count if GPS_count != 0 else np.nan,
        'GALILEO_avg': GALILEO_avg/GALILEO_count if GALILEO_count != 0 else np.nan,
        'BEIDOU_avg': BEIDOU_avg/BEIDOU_count if BEIDOU_count != 0 else np.nan,
        'GLONASS_avg': GLONASS_avg/GLONASS_count if GLONASS_count != 0 else np.nan
    })
    return data

def extract_mon_rf(ubx_msg,itow):
    """
    Iz UBX-NAV-PVT sporočila vrne ključne navigacijske podatke.
    """
    data = {                
        'jammingState_01': ubx_msg.jammingState_01,
        'jammingState_02': ubx_msg.jammingState_02,
        'jamInd_01': ubx_msg.jamInd_01,
        'jamInd_02': ubx_msg.jamInd_02,
        'iTOW': itow
    }
    return data

def extract_nav_timeutc(ubx_msg):
    return {
        'iTOW': ubx_msg.iTOW,
        'year': ubx_msg.year,
        'month': ubx_msg.month,
        'day': ubx_msg.day,
        'hour': ubx_msg.hour,
        'min': ubx_msg.min,
        'sec': ubx_msg.sec
    }

# Inicializacija
port = '/dev/ttyACM0'
data_buffer = {}
keys = set()
current_itow = None
start_time = None
current_df_itow = None

# Funkcija za pošiljanje podatkov preko API-ja
def send_data_to_api(df):
    try:
        json_data = df.to_json(orient='records')
        response = requests.post(API_URL, json=json.loads(json_data))
        print(f"API odgovor: {response.status_code}, {response.text}")
    except Exception as e:
        print(f"Napaka pri pošiljanju podatkov: {e}")


with serial.Serial(port, 38400, timeout=1) as ser:
    set_baudrate_ublox(ser)
    time.sleep(0.1)
# Odpri serijski port

with serial.Serial(port, 115200, timeout=1) as ser:
    ubr = UBXReader(ser, protfilter=2)
    configure_receiver(ser)

    try:
        while True:
            raw_data, parsed_data = next(ubr)
            data = {}

            # Preveri vrsto sporočila
            if parsed_data.identity == "NAV-SAT":
                data = extract_nav_sat(parsed_data)
                current_itow = data['iTOW']
            elif parsed_data.identity == "NAV-TIMEUTC":
                data = extract_nav_timeutc(parsed_data)
                current_itow = data['iTOW']
            elif parsed_data.identity == "MON-RF":
                data = extract_mon_rf(parsed_data,current_itow)
                current_itow = data['iTOW']

            if current_itow is None:
                continue

            # Pretvori iTOW v sekunde
            iTOW_sec = int(current_itow / 1000)
            data['iTOW_sec'] = iTOW_sec

            # Posodobi buffer
            if iTOW_sec not in data_buffer:
                data_buffer[iTOW_sec] = {}
            data_buffer[iTOW_sec].update(data)
            keys.update(data.keys())

            if start_time is None:
                start_time = iTOW_sec
                current_df_itow = start_time

            # Preveri, ali je preteklo 10 sekund
            if iTOW_sec >= current_df_itow + 10:
                df = pd.DataFrame(columns=['iTOW_sec','GPS_avg','GALILEO_avg','BEIDOU_avg','GLONASS_avg'] + list(keys - {'iTOW_sec','GPS_avg','GALILEO_avg','BEIDOU_avg','GLONASS_avg'}))

                # Dodaj podatke v DataFrame
                for sec in sorted(data_buffer.keys()):
                    if sec < current_df_itow + 10:
                        row = {key: data_buffer[sec].get(key, None) for key in df.columns}
                        df = pd.concat([df, pd.DataFrame([row])], ignore_index=True)

                # Izpiši dan, mesec, leto in zadnjo uro
                last_row = df.iloc[-1]
                print(f"Datum: {last_row['day']:02}.{last_row['month']:02}.{last_row['year']} {last_row['hour']:02}:{last_row['min']:02}:{last_row['sec']:02}")

                # Pošlji podatke preko API-ja
                #send_data_to_api(df)
                print(df.head())

                # Počisti buffer
                data_buffer = {k: v for k, v in data_buffer.items() if k >= current_df_itow + 10}
                keys = set()
                current_df_itow += 10

    except KeyboardInterrupt:
        print("Program prekinjen.")
    except Exception as e:
        print(f"Napaka: {e}")
