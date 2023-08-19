import PySimpleGUI as sg
import pickle

class DiveForm:
    def __init__(self, file_name):

        self.layout = [
            [sg.Text("Operator name:"), sg.InputText('',key='operator_name', size=(80, 1))],
            [sg.Text("Company name:"), sg.InputText('',key='company_name', size=(80, 1))],
            [sg.Text("Farm ID:"), sg.InputText('',key='farm_id', size=(80, 1))],
            [sg.Text("Description:"), sg.Multiline('',key='description', size=(80, 8))],
            [sg.Button("OK")]
        ]

        self.file_name = file_name
        self.values=None

    def open(self):
        window = sg.Window("Dive Form", self.layout)
        event, values = window.read(timeout=0)
        try:
            with open(self.file_name, "rb") as f:
                self.values=pickle.load(f)
                for fld in self.values:
                    if fld in window.AllKeysDict:
                        window[fld].update(self.values[fld])
                    else:
                        print('Error: not compatible field',fld)
        except FileNotFoundError:
            pass


        while True:
            event, values = window.read()
            if event == "OK":
                self.values=values
                self.save()
                break
            if event == sg.WIN_CLOSED:
                break
        window.close()

    def get(self):
        return self.values

    def save(self):
        with open(self.file_name, "wb") as f:
            pickle.dump(self.values, f,protocol=4)


if __name__=='__main__':
    df=DiveForm('/tmp/test.pkl')
    df.open()
    print(df.get())

