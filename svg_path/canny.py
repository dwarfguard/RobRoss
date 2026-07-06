import os
import cv2

APPLE_PATH = os.path.join(os.path.dirname(__file__), '..', 'apple.png')

img = cv2.imread(APPLE_PATH, cv2.IMREAD_GRAYSCALE)
edges = cv2.Canny(img, threshold1=100, threshold2=200) # Canny算子一行搞定pip install opencv-python

OUTPUT_PATH = os.path.join(os.path.dirname(__file__), 'apple_edges.png')
cv2.imwrite(OUTPUT_PATH, edges)

contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

height, width = edges.shape
svg_paths = []
for contour in contours:
    epsilon = 0.002 * cv2.arcLength(contour, closed=False)
    approx = cv2.approxPolyDP(contour, epsilon, closed=False)
    points = approx.reshape(-1, 2)
    if len(points) < 2:
        continue
    d = f"M {points[0][0]} {points[0][1]} " + " ".join(f"L {x} {y}" for x, y in points[1:])
    svg_paths.append(f'<path d="{d}" stroke="black" fill="none" stroke-width="1"/>')

svg_content = (
    f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
    f'viewBox="0 0 {width} {height}">\n'
    + "\n".join(svg_paths)
    + "\n</svg>"
)

SVG_PATH = os.path.join(os.path.dirname(__file__), 'apple_edges.svg')
with open(SVG_PATH, 'w') as f:
    f.write(svg_content)