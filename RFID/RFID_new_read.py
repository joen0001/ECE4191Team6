from mfrc522 import BasicMFRC522

reader = BasicMFRC522()
#sectors = [11, 15]
while True:
    id = reader.read_id()
    print(f"ID: {id}")
    #print(f"Text: {text}")