import json

class Args:

    def __init__(self, config_path):
        f = open(config_path)
        data = json.load(f)
        self.config_path = config_path
        self.hsv_blue = data["hsv_blue"]
        self.hsv_green = data["hsv_green"]
        self.hsv_orange = data["hsv_orange"]
        self.hsv_pink = data["hsv_pink"]

    def save(self):
        f = open(self.config_path, "w")
        data = {
            "hsv_blue": self.hsv_blue,
            "hsv_green": self.hsv_green,
            "hsv_orange": self.hsv_orange,
            "hsv_pink": self.hsv_pink
        }
        json.dump(data, f)