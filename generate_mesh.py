import math

def to_q24_8(val):
    """
    Converts a floating-point number to Q24.8 fixed-point format.
    - Multiplies by 256 (2^8)
    - Masks to 32 bits to handle Python's infinite precision negatives.
    """
    int_val = int(round(val * 256.0))
    return int_val & 0xFFFFFFFF

def rgb_to_565(r, g, b):
    """
    Converts standard 8-bit RGB (0-255) to 16-bit RGB565 format.
    """
    r5 = (r >> 3) & 0x1F
    g6 = (g >> 2) & 0x3F
    b5 = (b >> 3) & 0x1F
    return (r5 << 11) | (g6 << 5) | b5

def pack_vertex(x, y, z, w, r, g, b):
    """
    Packs X, Y, Z, W, and Color into a single 144-bit integer.
    Format: [143:112] X, [111:80] Y, [79:48] Z, [47:16] W, [15:0] RGB565
    """
    x_bits = to_q24_8(x)
    y_bits = to_q24_8(y)
    z_bits = to_q24_8(z)
    w_bits = to_q24_8(w)
    color_bits = rgb_to_565(r, g, b)
    
    # Shift values into their correct bit positions and combine them
    vertex_word = (x_bits << 112) | (y_bits << 80) | (z_bits << 48) | (w_bits << 16) | color_bits
    
    # Return as a 36-character hexadecimal string (144 bits / 4 bits per hex char = 36)
    return f"{vertex_word:036X}"

def main():
    # 1. Define the 8 corners of a 3D Cube
    # Size from -1.0 to 1.0 in 3D space. W is 1.0 for positions.
    v = [
        [-1.0, -1.0, -1.0, 1.0], [ 1.0, -1.0, -1.0, 1.0],
        [ 1.0,  1.0, -1.0, 1.0], [-1.0,  1.0, -1.0, 1.0],
        [-1.0, -1.0,  1.0, 1.0], [ 1.0, -1.0,  1.0, 1.0],
        [ 1.0,  1.0,  1.0, 1.0], [-1.0,  1.0,  1.0, 1.0]
    ]

    # 2. Define the 12 triangles (2 per face, 6 faces) using vertex indices
    # Each tuple is (v1, v2, v3, R, G, B)
    faces = [
        # Front face (Red)
        (0, 1, 2,  255, 0, 0), (0, 2, 3,  255, 0, 0),
        # Back face (Green)
        (5, 4, 7,  0, 255, 0), (5, 7, 6,  0, 255, 0),
        # Top face (Blue)
        (3, 2, 6,  0, 0, 255), (3, 6, 7,  0, 0, 255),
        # Bottom face (Yellow)
        (4, 5, 1,  255, 255, 0), (4, 1, 0,  255, 255, 0),
        # Right face (Magenta)
        (1, 5, 6,  255, 0, 255), (1, 6, 2,  255, 0, 255),
        # Left face (Cyan)
        (4, 0, 3,  0, 255, 255), (4, 3, 7,  0, 255, 255)
    ]

    # 3. Generate vertices.mem
    print("Generating vertices.mem...")
    with open("vertices.mem", "w") as f:
        for face in faces:
            idx1, idx2, idx3, r, g, b = face
            
            # Pack all three vertices of the triangle with the same face color
            f.write(pack_vertex(v[idx1][0], v[idx1][1], v[idx1][2], v[idx1][3], r, g, b) + "\n")
            f.write(pack_vertex(v[idx2][0], v[idx2][1], v[idx2][2], v[idx2][3], r, g, b) + "\n")
            f.write(pack_vertex(v[idx3][0], v[idx3][1], v[idx3][2], v[idx3][3], r, g, b) + "\n")

    # 4. Generate matrix.mem (Initial Identity Matrix)
    # An Identity matrix applies no rotation or scaling (V' = V). 
    # This is perfect for debugging the first static frame.
    identity_matrix = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ]

    print("Generating matrix.mem...")
    with open("matrix.mem", "w") as f:
        for row in identity_matrix:
            for val in row:
                q24_val = to_q24_8(val)
                f.write(f"{q24_val:08X}\n") # 32 bits = 8 hex chars

    print("Done! 'vertices.mem' and 'matrix.mem' created.")

if __name__ == "__main__":
    main()
