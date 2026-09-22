import os
from PIL import Image
from io import BytesIO

NUM_SEGMENTS = 22
CHUNK = 42                                                                  # Maximum size 46 (42+header)
REPS = 20                                                                    # Nº of segment repetition in the master file
SCALING = 5                                                                 # % of image scaling

def extract_and_prepend_chars(hex_data, start_pos=188, end_pos=196):
    start_of_scan = hex_data.find('ffda')
    if start_of_scan != -1:
        header = hex_data[:start_of_scan + 19]
        relevant_data = hex_data[start_of_scan + 19:]
        if len(header) >= end_pos:
            chars_to_prepend = header[start_pos:end_pos]                    # Extraer los caracteres 189 a 196 (índices 188 a 196)
            new_hex_data = chars_to_prepend + relevant_data
            return new_hex_data
        else:
            print("Encabezado más corto de lo esperado. No se extrajeron caracteres.")
            return hex_data[start_of_scan + 19:]
    else:
        print("No se encontró 'ffda' en los datos hexadecimales.")
        return hex_data

def segment_image(image, output_dir, width, height, chunk_size):
    segment_width = width // NUM_SEGMENTS
    os.makedirs(output_dir, exist_ok=True)
    all_hex_file_path = os.path.join(output_dir, "all_segments_hex.txt")    # Archivo maestro
    all_fragments = []
    for i in range(NUM_SEGMENTS):
        left = i * segment_width
        right = (i + 1) * segment_width if i != NUM_SEGMENTS - 1 else width # El último segmento toma el resto
        segment = image.crop((left, 0, right, height)).convert('L')         # Recortar y convertir a escala de grises
        segment_data = BytesIO()
        segment.save(segment_data, format='JPEG')
        hex_data = segment_data.getvalue().hex()
        modified_hex_data = extract_and_prepend_chars(hex_data)
        hex_lines = [modified_hex_data[j:j + chunk_size] for j in range(0, len(modified_hex_data), chunk_size)]
        with open(f"{output_dir}/segment_{i + 1}.txt", 'w') as f:
            for line_num, line in enumerate(hex_lines):
                line = line.ljust(chunk_size, 'f')  # Rellenar con 'f' si es necesario
                header = f"{i + 1:02X}{line_num + 1:02X}{line}"
                if line_num == len(hex_lines) - 1:
                    f.write(header)
                else:
                    f.write(header + "\n")
        for line_num, line in enumerate(hex_lines):
            line = line.ljust(chunk_size, 'f')
            header = f"{i + 1:02X}{line_num + 1:02X}{line}"
            all_fragments.append(header)
    with open(all_hex_file_path, 'w') as all_hex_file:
        for _ in range(REPS):
            for idx, fragment_line in enumerate(all_fragments):
                if idx == len(all_fragments) - 1 and _ == REPS - 1:
                    all_hex_file.write(fragment_line)
                else:
                    all_hex_file.write(fragment_line + "\n")
    with open(all_hex_file_path, 'r') as file:
            lines = file.readlines()
    with open(all_hex_file_path, 'w') as all_hex_file:
        for idx, line in enumerate(lines):
            if idx == len(lines) - 1:
                all_hex_file.write(f"{idx}:{line.strip()}")
            else:
                all_hex_file.write(f"{idx}:{line}")

#image_path = "image_pop_up.jpg"
image_path=str(input('Paste your image path here: '))
segment_dir = "segments"
image = Image.open(image_path)
width, height = image.size
image_width, image_height = int(width * SCALING / 100), int(height * SCALING / 100)
resized_image = image.resize((image_width, image_height))
segment_image(resized_image, segment_dir, image_width, image_height, CHUNK)