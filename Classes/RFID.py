from mfrc522 import BasicMFRC522
A1 = 584193570396
A2 = 584193308248
A3 = 584195405368
A4 = 584195667516

B1 = 584191014023
B2 = 584191276155
B3 = 584188458401
B4 = 584188720549

C1 = 584196650284
C2 = 584196388136
C3 = 584183936490
C4 = 584184198638
class RFIDScanner:
    def __init__(self):
        self.reader = BasicMFRC522()
    def BEEP(self):
        List_A = [A1,A2,A3,A4]
        List_B = [B1,B2,B3,B4]
        List_C = [C1,C2,C3,C4]
        Loc_Ind = ["A","B","C"]
        List = [List_A,List_B,List_C]

        id = self.reader.read_id()

        for letter in range(3):
            for ind in range(4):
                if id == List[letter][ind]:
                    Loc = Loc_Ind[letter]
        return Loc