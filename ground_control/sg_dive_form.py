import PySimpleGUI as sg
import json

class DiveForm:
    def __init__(self, file_name):
        self.fields = {"full_name": "", "description": ""}
        self.file_name = file_name
        try:
            with open(self.file_name, "r") as f:
                self.fields = json.load(f)
        except FileNotFoundError:
            pass

    def open(self):
        layout = [
            [sg.Text("Full name:"), sg.InputText(self.fields["full_name"], size=(80, 1))],
            [sg.Text("Description:"), sg.Multiline(self.fields["description"], size=(80, 8))],
            [sg.Button("OK")]
        ]

        window = sg.Window("Dive Form", layout)

        while True:
            event, values = window.read()
            if event == "OK":
                self.fields["full_name"] = values[0]
                self.fields["description"] = values[1]
                self.save()
                return self.fields
            if event == sg.WIN_CLOSED:
                break
        window.close()

    def get(self):
        return self.fields

    def save(self):
        with open(self.file_name, "w") as f:
            json.dump(self.fields, f)
