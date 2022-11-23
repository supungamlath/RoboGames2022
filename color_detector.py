class ColorDetector:
    def __init__(self, camera):
        self.camera = camera
        self.hex_to_names = {
            "#00ffff": "aqua",
            "#000000": "black",
            "#0000ff": "blue",
            "#ff00ff": "fuchsia",
            "#008000": "green",
            "#808080": "grey",
            "#00ff00": "lime",
            "#800000": "maroon",
            "#000080": "navy",
            "#808000": "olive",
            "#800080": "purple",
            "#ff0000": "red",
            "#c0c0c0": "silver",
            "#008080": "teal",
            "#ffffff": "white",
            "#ffff00": "yellow",
        }

    def hex_to_rgb(self, hex_value):
        return tuple(
            map(
                lambda s: int(s, 16),
                (hex_value[1:3], hex_value[3:5], hex_value[5:7]),
            )
        )

    def closestColour(self, requested_colour):
        min_colours = {}
        for key, name in self.hex_to_names.items():
            r_c, g_c, b_c = self.hex_to_rgb(key)
            rd = (r_c - requested_colour[0]) ** 2
            gd = (g_c - requested_colour[1]) ** 2
            bd = (b_c - requested_colour[2]) ** 2
            min_colours[(rd + gd + bd)] = name
        return min_colours[min(min_colours.keys())]

    def testColorInCameraRow(self, test_colors, row):
        image = self.camera.getImage()
        width = self.camera.getWidth()
        blue, green, red = 0, 0, 0
        for w in range(width):
            blue += self.camera.imageGetBlue(image, width, w, row)
            green += self.camera.imageGetGreen(image, width, w, row)
            red += self.camera.imageGetRed(image, width, w, row)
        color_name = self.closestColour((red // width, green // width, blue // width))

        return color_name in test_colors
