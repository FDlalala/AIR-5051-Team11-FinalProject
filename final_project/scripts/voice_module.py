import torch
from transformers import MarianMTModel, MarianTokenizer
from faster_whisper import WhisperModel

# Whisper 初始化（本地模型）
local_model_path = "/home/robotics/sagittarius_ws/src/final_project/models/faster-whisper-large-v3"
whisper_model = WhisperModel(local_model_path, device="cuda" if torch.cuda.is_available() else "cpu", local_files_only=True)

# 翻译模型本地路径
en_zh_path = "/home/robotics/sagittarius_ws/src/final_project/models/Helsinki-NLP/opus-mt-en-zh"
de_zh_path = "/home/robotics/sagittarius_ws/src/final_project/models/Helsinki-NLP/opus-mt-de-zh"

en_tokenizer = MarianTokenizer.from_pretrained(en_zh_path, local_files_only=True)
en_model = MarianMTModel.from_pretrained(en_zh_path, local_files_only=True)

de_tokenizer = MarianTokenizer.from_pretrained(de_zh_path, local_files_only=True)
de_model = MarianMTModel.from_pretrained(de_zh_path, local_files_only=True)

# 翻译为中文
def translate_to_zh(text, lang):
    if lang == "en":
        tokenizer, model = en_tokenizer, en_model
        return decode_translation(text, tokenizer, model), 2
    elif lang == "de":
        tokenizer, model = de_tokenizer, de_model
        return decode_translation(text, tokenizer, model), 3
    else:
        return text, 1

def decode_translation(text, tokenizer, model):
    inputs = tokenizer(text, return_tensors="pt", padding=True)
    outputs = model.generate(**inputs)
    return tokenizer.decode(outputs[0], skip_special_tokens=True)

# 中文指令解析函数
def parse_command(text):
    if "红" in text:
        return "红", 1
    elif "绿" in text:
        return "绿", 2
    elif "蓝" in text:
        return "蓝", 3
    else:
        return "未识别", 0

# 主识别函数：识别 + 翻译 + 匹配颜色
def recognize_from_wav(wav_path):
    segments, info = whisper_model.transcribe(wav_path)
    text = "".join([seg.text for seg in segments]).strip()
    print("识别文本：", text)
    print("检测语言：", info.language)
    if info.language in ["en", "de"]:
        translated, _ = translate_to_zh(text, info.language)
        print("翻译为中文：", translated)
        color, code = parse_command(translated)
    else:
        color, code = parse_command(text)
    print("物体颜色：", color,  "\n=================================")
    return color, code


