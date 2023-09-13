from mfrc522 import MFRC522

reader = MFRC522()

def write(trailer_block, key, block_addrs, text):
    (status, TagType) = reader.Request(reader.PICC_REQIDL)
    if status != reader.MI_OK:
        return None, None
    (status, uid) = reader.Anticoll()
    if status != reader.MI_OK:
        return None, None
    reader.SelectTag(uid)
    status = reader.Authenticate(
        reader.PICC_AUTHENT1A, trailer_block, key, uid)
    reader.ReadTag(trailer_block)
    if status == reader.MI_OK:
        data = bytearray()
        data.extend(bytearray(text.ljust(
            len(block_addrs) * 16).encode('ascii')))
        i = 0
        for block_num in block_addrs:
            reader.WriteTag(block_num, data[(i*16):(i+1)*16])
            i += 1
    reader.StopAuth()
    return uid, text[0:(len(block_addrs) * 16)]

trailer_block = 11
block_addrs = [8, 9, 10]
key = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
text = "some random text"
uid, text_in = write(trailer_block, key, block_addrs, text)
while not uid:
    uid, text_in = write(trailer_block, key, block_addrs, text)
print(uid)
print(text_in)