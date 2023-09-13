from mfrc522 import BasicMFRC522

reader = BasicMFRC522()
sectors = [11, 15]
id, text = reader.read_sectors(sectors)
print(f"ID: {id}")
print(f"Text: {text}")