import struct

def float_to_fp32_hex(f):
    """Converts a float to a 32-bit IEEE 754 (FP32) hex string."""
    packed = struct.pack('>f', f)
    ival = struct.unpack('>I', packed)[0]
    return f"{ival:08x}"

def float_to_q24_8_hex(val):
    """Converts a float to a 32-bit Q24.8 fixed-point hex string."""
    fixed_val = int(val * 256)
    if fixed_val < 0:
        fixed_val = fixed_val & 0xFFFFFFFF # Handle Two's Complement
    return f"{fixed_val:08x}"

def rgb_to_rgb565_hex(r, g, b):
    """Packs 8-bit RGB channels into a 16-bit RGB565 hex string."""
    r5 = (r >> 3) & 0x1F
    g6 = (g >> 2) & 0x3F
    b5 = (b >> 3) & 0x1F
    val = (r5 << 11) | (g6 << 5) | b5
    return f"{val:04x}"

def generate_vertices_mem(filename="vertices.mem"):
    """
    Generates a 3D cube (36 vertices) and writes them line-by-line.
    Packing Strategy: {X[31:0], Y[31:0], Z[31:0], W[31:0], R[4:0], G[5:0], B[4:0]}
    """
    # Define cube corners
    p0 = (-0.5, -0.5, -0.5)
    p1 = ( 0.5, -0.5, -0.5)
    p2 = ( 0.5,  0.5, -0.5)
    p3 = (-0.5,  0.5, -0.5)
    p4 = (-0.5, -0.5,  0.5)
    p5 = ( 0.5, -0.5,  0.5)
    p6 = ( 0.5,  0.5,  0.5)
    p7 = (-0.5,  0.5,  0.5)

    # 12 Triangles (2 per face) with specific RGB colors
    triangles = [
        (p4, p5, p6, (255, 0, 0)),   (p4, p6, p7, (255, 0, 0)),   # Front (Red)
        (p1, p0, p3, (0, 255, 255)), (p1, p3, p2, (0, 255, 255)), # Back (Cyan)
        (p0, p4, p7, (0, 255, 0)),   (p0, p7, p3, (0, 255, 0)),   # Left (Green)
        (p5, p1, p2, (255, 0, 255)), (p5, p2, p6, (255, 0, 255)), # Right (Magenta)
        (p7, p6, p2, (0, 0, 255)),   (p7, p2, p3, (0, 0, 255)),   # Top (Blue)
        (p0, p1, p5, (255, 255, 0)), (p0, p5, p4, (255, 255, 0))  # Bottom (Yellow)
    ]

    with open(filename, "w") as f:
        for tri in triangles:
            color = tri[3]
            hcol = rgb_to_rgb565_hex(*color)
            
            # Write 3 vertices per triangle
            for i in range(3):
                x, y, z = tri[i]
                w = 1.0
                
                hx = float_to_fp32_hex(x)
                hy = float_to_fp32_hex(y)
                hz = float_to_fp32_hex(z)
                hw = float_to_fp32_hex(w)
                
                # Combine exactly as defined: {X, Y, Z, W, RGB565}
                vertex_hex = f"{hx}{hy}{hz}{hw}{hcol}"
                f.write(vertex_hex + "\n")
                
    print(f"Generated {filename} (36 lines, 144-bits each).")

def generate_matrix_mem(filename="matrix.mem"):
    """
    Generates a 4x4 Identity Matrix in Q24.8 format.
    Written as 16 separate 32-bit lines so $readmemh can load it 
    into the reg [31:0] matrix_regs [0:15] array.
    """
    matrix = [
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    ]
    
    with open(filename, "w") as f:
        for element in matrix:  # Corrected from "isn" to "in"
            f.write(float_to_q24_8_hex(element) + "\n")
            
    print(f"Generated {filename} (16 lines, 32-bits each).")

if __name__ == "__main__":
    generate_vertices_mem("vertices.mem")
    generate_matrix_mem("matrix.mem")
