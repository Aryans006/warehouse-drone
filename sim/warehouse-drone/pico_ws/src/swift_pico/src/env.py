from PIL import Image, ImageDraw

class Env:
    def __init__(self, image_path):
        self.x_range = 1000  # Background width
        self.y_range = 1000  # Background height
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.obs = self.obs_map(image_path)

    def obs_map(self, image_path):
        # Open the image and resize if necessary
        img = Image.open(image_path).convert("1")  # Convert to binary (black and white)
        img = img.resize((self.x_range, self.y_range))  # Ensure it's 1000x1000

        obs = set()
        # Loop through each pixel
        for x in range(self.x_range):
            for y in range(self.y_range):
                # Add pixel coordinates to obs if the pixel is black
                if img.getpixel((x, y)) == 0:  # 0 means black in binary mode
                    obs.add((x, y))
        return obs

    def draw_env(self, point_size=10):
        # Create a blank white background image
        img = Image.new("RGB", (self.x_range, self.y_range), "white")
        draw = ImageDraw.Draw(img)

        # Draw obstacles as squares
        for x, y in self.obs:
            draw.rectangle(
                [(x - point_size // 2, y - point_size // 2), 
                 (x + point_size // 2, y + point_size // 2)], 
                fill="black"
            )

        return img

# Initialize environment and draw it with the uploaded image as obstacle map
# env = Env("/home/aryan/eyantra/warehouse-drone/pico_ws/src/swift_pico/src/image.png")
# image = env.draw_env()
# image.show()
