from PIL import Image
from io import BytesIO
import os

NUM_SEGMENTS = 22
CHUNK_SIZE = 42                                                             # Maximum size 46 (42+header)
HEADER_SIZE = 4                                                             # Header size
LINES_PER_FRAGMENT = 50                                                     # Nº lines per segment (needs to be adjusted yet)

def parse_hex_file(hex_file):
    segments = {i: [] for i in range(1, NUM_SEGMENTS + 1)}                  # Diccionary for segment data
    seen_lines = set()                                                      # Detecting duplicates
    with open(hex_file, 'r') as file:
        for line in file:
            line = line.split(":", 1)[-1].strip()
            header = line[:HEADER_SIZE]
            fragment = int(header[:2], 16)                                  # 2 first digits of the header are the segment
            line_number = int(header[2:], 16)                               # 2 last digits of the header are the line
            data = line[HEADER_SIZE:].strip()                               # Remaining digits are image data
            unique_key = (fragment, line_number, data)
            if unique_key not in seen_lines:
                seen_lines.add(unique_key)
                if fragment in segments:
                    segments[fragment].append((line_number, data))          # Storing lines in their segments
    for fragment in segments:
        segments[fragment].sort(key=lambda x: x[0])                         # Arrange lines inside each segment
    return segments

def verificar_lineas_faltantes(segment, fragment_id):
    lineas_presentes = [line_number for line_number, _ in segment]
    lineas_faltantes = [i for i in range(LINES_PER_FRAGMENT) if i not in lineas_presentes]
    #if lineas_faltantes:
        #print(f"Líneas faltantes en el fragmento {fragment_id}: {lineas_faltantes}")
    return lineas_faltantes

def reconstruct_image(hex_file, output_image, image_width, image_height, header_file="header.txt", output_dir="segment_images"):
    os.makedirs(output_dir, exist_ok=True)
    segment_width = image_width // NUM_SEGMENTS
    reconstructed_image = Image.new('L', (image_width, image_height))       # Creating image in greyscale
    with open(header_file, 'r') as file:
        header_template = file.read().strip()
    segments = parse_hex_file(hex_file)                                     # Organizing data per segment and line
    for i in range(1, NUM_SEGMENTS + 1):
        left = (i - 1) * segment_width
        right = i * segment_width if i != NUM_SEGMENTS else image_width     # Last segment size
        if segments[i]:                                                
            first_data_line = segments[i][0][1]
            index, first_data_line = segments[i][0]
            updated_header = header_template.replace("ZZZZZZZZZ", first_data_line[:9])
            new_trama = first_data_line[9:]
            segments[i] = [(index, new_trama)] + segments[i][1:]
            segment_data = updated_header + ''.join([data for _, data in segments[i]])
        else:
            segment_data = header_template.replace("ZZZZZZZZZ", "000000000")
        lineas_faltantes = verificar_lineas_faltantes(segments[i], i)
        if lineas_faltantes:
            for missing_line in lineas_faltantes:
                segment_data += '00' * (CHUNK_SIZE // 2)                    # Asuming pairs of hexadecimal data
        end_index = segment_data.find('ffd9')
        if end_index != -1:
            segment_data = segment_data[:end_index + 4]
        try:
            segment_bytes = bytes.fromhex(segment_data)                     # Reconstruct hex, even inf markers are lost
            segment = Image.open(BytesIO(segment_bytes)).convert('L')
        except (ValueError, OSError):                                       # ONLY in case of error, print a blank segment
            segment = Image.new('1', (right - left, image_height))
            print(f"Segment {i} is corrupted or missing, replaced with a color segment.")
        segment_width_actual = right - left                                 # Verify and adjust segment size
        if segment.size != (segment_width_actual, image_height):
            print(f"Resizing segment {i} from {segment.size} to {(segment_width_actual, image_height)}")
            segment = segment.resize((segment_width_actual, image_height))
        segment_output_path = os.path.join(output_dir, f"segment_{i}.jpg")
        segment.save(segment_output_path)
        print(f"Segment {i} saved as {segment_output_path}")
        reconstructed_image.paste(segment, (left, 0, right, image_height))
    reconstructed_image.save(output_image)
    print(f"Reconstructed image saved as {output_image}")

#hex_file = "segments/all_segments_hex.txt"
hex_file=str(input('Paste your "all_segments_hex.txt" path here: '))
final_image_path = "reconstructed_image.jpg"
image_width, image_height = 1156, 868                                       # Output line dimensions
reconstruct_image(hex_file, final_image_path, image_width, image_height)