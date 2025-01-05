import serial
import pandas as pd
import requests
import json
import time
from pyubx2 import UBXReader, UBXMessage
import warnings

warnings.simplefilter(action='ignore', category=FutureWarning)

# URL API strežnika
API_URL = "http://192.168.1.123:5000/api/test"

# Funkcija za vklop NAV-SVINFO, NAV-CLOCK, NAV-POSLLH in NAV-TIMEUTC
def enable_nav_msgs(ser):
    msg_svinfo = UBXMessage('CFG', 'CFG-MSG', msgmode=1, SET=1, msgClass=0x01, msgID=0x30, rateUART1=1)
    ser.write(msg_svinfo.serialize())

    msg_clock = UBXMessage('CFG', 'CFG-MSG', msgmode=1, SET=1, msgClass=0x01, msgID=0x22, rateUART1=1)
    ser.write(msg_clock.serialize())

    msg_posllh = UBXMessage('CFG', 'CFG-MSG', msgmode=1, SET=1, msgClass=0x01, msgID=0x02, rateUART1=1)
    ser.write(msg_posllh.serialize())

    msg_timeutc = UBXMessage('CFG', 'CFG-MSG', msgmode=1, SET=1, msgClass=0x01, msgID=0x21, rateUART1=1)
    ser.write(msg_timeutc.serialize())

# Funkcije za ekstrakcijo podatkov
def extract_svinfo(ubx_msg):
    data = {}
    for i in range(1, ubx_msg.numCh + 1):
        sv_id = getattr(ubx_msg, f'svid_{i:02}')
        cno = getattr(ubx_msg, f'cno_{i:02}')
        data[f'sv_{sv_id}_cno'] = cno
    return data

def extract_nav_clock(ubx_msg):
    return {
        'iTOW': ubx_msg.iTOW,
        'clkB': ubx_msg.clkB,
        'clkD': ubx_msg.clkD,
        'tAcc': ubx_msg.tAcc,
        'fAcc': ubx_msg.fAcc
    }

def extract_nav_posllh(ubx_msg):
    return {
        'iTOW': ubx_msg.iTOW,
        'lon': ubx_msg.lon,
        'lat': ubx_msg.lat,
        'height': ubx_msg.height / 1000,
        'hMSL': ubx_msg.hMSL / 1000,
        'hAcc': ubx_msg.hAcc / 1000,
        'vAcc': ubx_msg.vAcc / 1000
    }

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
port = '/dev/ttyUSB0'
baudrate = 9600
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

# Odpri serijski port
with serial.Serial(port, baudrate, timeout=1) as ser:
    ubr = UBXReader(ser, protfilter=2)
    enable_nav_msgs(ser)

    try:
        while True:
            raw_data, parsed_data = next(ubr)
            data = {}

            # Preveri vrsto sporočila
            if parsed_data.identity == "NAV-SVINFO":
                data = extract_svinfo(parsed_data)
            elif parsed_data.identity == "NAV-CLOCK":
                data = extract_nav_clock(parsed_data)
                current_itow = data['iTOW']
            elif parsed_data.identity == "NAV-POSLLH":
                data = extract_nav_posllh(parsed_data)
                current_itow = data['iTOW']
            elif parsed_data.identity == "NAV-TIMEUTC":
                data = extract_nav_timeutc(parsed_data)
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
                df = pd.DataFrame(columns=['iTOW_sec'] + list(keys - {'iTOW_sec'}))

                # Dodaj podatke v DataFrame
                for sec in sorted(data_buffer.keys()):
                    if sec < current_df_itow + 10:
                        row = {key: data_buffer[sec].get(key, None) for key in df.columns}
                        df = pd.concat([df, pd.DataFrame([row])], ignore_index=True)

                # Izpiši dan, mesec, leto in zadnjo uro
                last_row = df.iloc[-1]
                print(f"Datum: {last_row['day']:02}.{last_row['month']:02}.{last_row['year']} {last_row['hour']:02}:{last_row['min']:02}:{last_row['sec']:02}")

                # Pošlji podatke preko API-ja
                send_data_to_api(df)

                # Počisti buffer
                data_buffer = {k: v for k, v in data_buffer.items() if k >= current_df_itow + 10}
                keys = set()
                current_df_itow += 10

    except KeyboardInterrupt:
        print("Program prekinjen.")
    except Exception as e:
        print(f"Napaka: {e}")
