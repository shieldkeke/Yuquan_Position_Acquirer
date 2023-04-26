
class Debugger():

    def __init__(self, file_name, mode = "w"):
        self.file_name = file_name
        self.mode = mode
        self.file = open(self.file_name, self.mode)

    def print(self, content):
        for element in content:
            self.file.write(str(element) + " ")
        self.file.write("\n")

    def close(self):
        self.file.close()
        
    def __del__(self):
        self.file.close()