import os
from pydub import AudioSegment

# 设置目标文件夹路径
input_dir = "/home/robotics/sagittarius_ws/src/final_project/test_data/colour"  # 修改为你的文件夹路径

# 遍历目录下所有文件
for filename in os.listdir(input_dir):
    if filename.endswith(".m4a"):
        m4a_path = os.path.join(input_dir, filename)
        wav_path = os.path.join(input_dir, filename.replace(".m4a", ".wav"))
        print(f"正在转换: {m4a_path} -> {wav_path}")
        audio = AudioSegment.from_file(m4a_path, format="m4a")
        audio.export(wav_path, format="wav")
print("全部转换完成。")
