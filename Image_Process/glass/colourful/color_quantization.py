import numpy as np
from PIL import Image
import tkinter as tk
from tkinter import filedialog
import os

def replace_with_weighted_palette(input_path, output_path, target_size=(512, 512)):
    # 1. 定义标准色块库以及对应的“惩罚权重”
    palette_data = {
        "Black":  {"color": (0, 0, 0),       "weight": 1.0}, # 优先
        "White":  {"color": (255, 255, 255), "weight": 1.0}, 
        "Red":    {"color": (255, 0, 0),     "weight": 1.0}, 
        "Yellow": {"color": (255, 255, 0),   "weight": 1.0}, 
        "Blue":   {"color": (0, 0, 255),     "weight": 1.0}, 
        "Gray":   {"color": (128, 128, 128), "weight": 2.0}, # 降级：距离放大2倍
        "Orange": {"color": (255, 165, 0),   "weight": 2.0}, 
        "Green":  {"color": (0, 128, 0),     "weight": 2.0}, 
        "Purple": {"color": (128, 0, 128),   "weight": 2.0}  
    }
    
    colors_list = [data["color"] for data in palette_data.values()]
    weights_list = [data["weight"] for data in palette_data.values()]
    
    palette_colors = np.array(colors_list)
    color_weights = np.array(weights_list)

    # 2. 读取图像、转换为纯RGB模式并调整大小
    try:
        img = Image.open(input_path).convert('RGB')
    except Exception as e:
        print(f"无法读取图片: {e}")
        return
        
    img_resized = img.resize(target_size, Image.Resampling.LANCZOS)
    img_array = np.array(img_resized)
    height, width, channels = img_array.shape
    
    # 3. 计算带权重的颜色距离
    pixels = img_array.reshape(-1, 1, 3)          
    colors = palette_colors.reshape(1, -1, 3)     
    weights = color_weights.reshape(1, -1)        
    
    base_distances = np.sum((pixels - colors) ** 2, axis=2)
    weighted_distances = base_distances * weights
    
    closest_color_indices = np.argmin(weighted_distances, axis=1)
    
    new_pixels = palette_colors[closest_color_indices]
    new_img_array = new_pixels.reshape(height, width, channels).astype(np.uint8)
    
    # 4. 保存图像
    result_img = Image.fromarray(new_img_array, 'RGB')
    result_img.save(output_path)
    print(f"处理完成！图片已保存至: {output_path}")

# ================= 使用示例 =================
if __name__ == "__main__":
    # 初始化 tkinter，但隐藏主窗口
    root = tk.Tk()
    root.withdraw()
    
    print("请在弹出的窗口中选择你要处理的图片...")
    
    # 弹出文件选择对话框
    input_image_path = filedialog.askopenfilename(
        title="选择一张图片",
        filetypes=[("Image files", "*.jpg *.jpeg *.png *.bmp *.webp"), ("All files", "*.*")]
    )
    
    if not input_image_path:
        print("未选择任何文件，程序退出。")
    else:
        print(f"已选择图片: {input_image_path}")
        print("正在处理，请稍候...")
        
        # 自动生成输出路径 (在原文件名的基础上加上 _quantized)
        file_dir, file_name = os.path.split(input_image_path)
        file_base, file_ext = os.path.splitext(file_name)
        output_image_path = os.path.join(file_dir, f"{file_base}_quantized.png")
        
        # 固定分辨率大小 (宽度, 高度)
        TARGET_RESOLUTION = (800, 800) 
        
        # 运行处理函数
        replace_with_weighted_palette(input_image_path, output_image_path, TARGET_RESOLUTION)