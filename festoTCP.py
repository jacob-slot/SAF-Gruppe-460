import socket

def start_server(host='172.20.66.159', port=10):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()
        print(f"Server listening on {host}:{port}")
        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")
            while True:
                
                # if connection is broken, break
                if not conn:
                    break

                # receive string data from the client and save as xml file
                data = conn.recv(1024)
                
                # remove all characters after </data> tag
                data = data[:data.find(b'</data>')+7]
                
                print("Received:", data)
                with open("data.xml", "wb") as f:
                    f.write(data)
                
                station_id = int(data[data.find(b'<station_id>')+12:data.find(b'</station_id>')].decode('utf-8'))
                print("Station ID:", station_id)

                carrier_id = int(data[data.find(b'<carrier_id>')+12:data.find(b'</carrier_id>')].decode('utf-8'))
                print("Carrier ID:", carrier_id)

                # open processing_times_table.csv with separator ;, and find the processing time for the column nr. station_id and line nr. carrier_id
                with open("processing_times_table.csv", "r") as f:
                    lines = f.readlines()
                    processing_time = int(lines[carrier_id].split(';')[station_id])
                    print("Processing time:", processing_time)

                conn.sendall((processing_time).to_bytes(2, byteorder='little', signed=True))

if __name__ == "__main__":
    while True:
        start_server()
