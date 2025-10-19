"""
语音唤醒助手 - Voice Wake Assistant
功能：
1. 持续监听语音，检测唤醒词
2. 唤醒后自动进入对话模式
3. 对话超时后自动休眠
"""
import os
import time
import threading
from collections import deque

from functions import func_map

# 初始化所有模型
from spacemit_llm import LLMModel, FCModel
from spacemit_asr import ASRModel
from spacemit_audio import RecAudioVadThread
from spacemit_tts import TTSModel, play_audio

# ==================== 配置参数 ====================

# 唤醒词配置（支持多个唤醒词）
WAKE_WORDS = [
    "小助手", "你好小助手", "嘿小助手", 
    "小爱", "你好小爱",
    "助手", "语音助手"
]

# 对话超时时间（秒），超过此时间无输入则进入休眠
DIALOG_TIMEOUT = 30

# 最大连续对话轮数
MAX_DIALOG_TURNS = 10

# 休眠提示语
SLEEP_PHRASES = [
    "我先休息了，需要时再叫我",
    "好的，有需要随时叫我",
    "那我先待机了"
]

print("="*70)
print("正在初始化语音唤醒助手...")
print("="*70)
init_start = time.time()

llm_model = LLMModel()
fc_model = FCModel()
asr_model = ASRModel()

recorder = RecAudioVadThread(
    sld=1,           # 静音1秒后停止
    max_time=5,      # 最大录音5秒
    channels=1,
    rate=48000,
    device_index=2,  # ⚠️ 需要根据实际设备修改
    trig_on=0.60,
    trig_off=0.35
)

tts_model = TTSModel()

tts_model.ort_predict("系统初始化")
init_time = time.time() - init_start
print(f" 所有模型初始化完成！耗时: {init_time:.2f}s")
print("="*70 + "\n")

class DialogState:
    def __init__(self):
        self.is_awake = False
        self.last_interaction_time = 0
        self.dialog_count = 0
        self.dialog_history = deque(maxlen=20)
    
    def wake_up(self):
        self.is_awake = True
        self.last_interaction_time = time.time()
        self.dialog_count = 0
        print("\n" + "="*70)
        print(" 助手已唤醒！")
        print("="*70 + "\n")
    
    def update_interaction(self):
        self.last_interaction_time = time.time()
        self.dialog_count += 1
    
    def should_sleep(self):
        if not self.is_awake:
            return False
        
        if time.time() - self.last_interaction_time > DIALOG_TIMEOUT:
            return True
        
        if self.dialog_count >= MAX_DIALOG_TURNS:
            return True
        
        return False
    
    def sleep(self):
        self.is_awake = False
        self.dialog_count = 0
        print("\n" + "="*70)
        print(" 助手进入休眠...")
        print("="*70 + "\n")
    
    def add_dialog(self, user_text, assistant_text):
        self.dialog_history.append({
            'user': user_text,
            'assistant': assistant_text,
            'timestamp': time.time()
        })


class PerformanceStats:
    def __init__(self):
        self.wake_times = []
        self.dialog_times = []
        self.total_dialogs = 0
    
    def add_wake(self, time_cost):
        self.wake_times.append(time_cost)
    
    def add_dialog(self, time_cost):
        self.dialog_times.append(time_cost)
        self.total_dialogs += 1
    
    def print_summary(self):
        if self.total_dialogs == 0:
            print(" 暂无对话统计")
            return
        
        print("\n" + "="*70)
        print(" 性能统计摘要")
        print("="*70)
        print(f"  总对话轮数: {self.total_dialogs}")
        print(f"  唤醒次数: {len(self.wake_times)}")
        if self.wake_times:
            print(f"  平均唤醒时间: {sum(self.wake_times)/len(self.wake_times):.2f}s")
        if self.dialog_times:
            print(f"  平均对话时间: {sum(self.dialog_times)/len(self.dialog_times):.2f}s")
        print("="*70)


def check_wake_word(text):
    text_lower = text.lower().strip()
    
    for wake_word in WAKE_WORDS:
        if wake_word in text_lower:
            return True
    
    text_clean = ''.join(c for c in text_lower if c.isalnum())
    for wake_word in WAKE_WORDS:
        wake_word_clean = ''.join(c for c in wake_word if c.isalnum())
        if wake_word_clean in text_clean:
            return True
    
    return False


def listen_for_wake_word(state):

    try:
        recorder.start_recording()
        recorder.stop_recording()
        audio_ret = recorder.get_audio()
        
        if audio_ret is None or len(audio_ret) == 0:
            return False
        
        text = asr_model.generate(audio_ret)
        
        if not text or text.strip() == "":
            return False
        
        if check_wake_word(text):
            print(f" 检测到唤醒词: {text}")
            return True
        return False
        
    except Exception as e:
        print(f"  唤醒检测异常: {e}")
        return False


def process_dialog_turn(state, stats):
    try:
        print("🎤 正在听你说话...")
        recorder.start_recording()
        recorder.stop_recording()
        audio_ret = recorder.get_audio()
        
        if audio_ret is None or len(audio_ret) == 0:
            print("  未检测到有效语音\n")
            return False
        
        start_time = time.time()
        
        print(" 识别中...", end='', flush=True)
        t_asr_start = time.time()
        text = asr_model.generate(audio_ret)
        t_asr = time.time() - t_asr_start
        print(f" ✓ ({t_asr:.2f}s)")
        
        if not text or text.strip() == "":
            print("  识别结果为空\n")
            return False
        
        print(f"👤 用户: {text}")
        
        exit_words = ['退下', '你退下吧', '休息吧', '去休息', '待机', '睡觉', '再见']
        if any(word in text for word in exit_words):
            feedback = SLEEP_PHRASES[0]
            print(f" 助手: {feedback}")
            try:
                audio_out = tts_model.ort_predict(feedback)
                play_audio(audio_out)
            except:
                pass
            state.sleep()
            return True
        
        print(" 分析意图...", end='', flush=True)
        t_fc_start = time.time()
        function_called = fc_model.func_response(text, func_map)
        t_fc = time.time() - t_fc_start
        print(f" ✓ ({t_fc:.2f}s)")
        
        if function_called:
            feedback = "好的，已经为你执行"
            print(f" 助手: {feedback}")
            
            try:
                audio_out = tts_model.ort_predict(feedback)
                t_tts = time.time() - t_fc_start
                play_audio(audio_out)
            except Exception as e:
                print(f"  [TTS失败: {e}]")
                t_tts = 0
            
            state.add_dialog(text, feedback)
            state.update_interaction()
            
            total_time = time.time() - start_time
            print(f"⏱  耗时: {total_time:.2f}s\n")
            stats.add_dialog(total_time)
            return True
        
        # 5. LLM对话
        print(" 思考中...", end='', flush=True)
        t_llm_start = time.time()
        llm_output = llm_model.generate(text)
        
        print("\r 助手: ", end='', flush=True)
        full_response = ""
        for output_text in llm_output:
            print(output_text, end='', flush=True)
            full_response += output_text
        
        t_llm = time.time() - t_llm_start
        print()
        
        if not full_response or full_response.strip() == "":
            print("  LLM未返回有效回复\n")
            return False
        
        print(" 合成语音...", end='', flush=True)
        try:
            t_tts_start = time.time()
            audio_out = tts_model.ort_predict(full_response)
            t_tts = time.time() - t_tts_start
            print(f"  ({t_tts:.2f}s)")
            
            print(" 播放中...")
            play_audio(audio_out)
        except Exception as e:
            print(f" ✗ (失败: {e})")
            t_tts = 0
        
        state.add_dialog(text, full_response)
        state.update_interaction()
        
        total_time = time.time() - start_time
        print(f"  耗时: {total_time:.2f}s (ASR:{t_asr:.2f}s + LLM:{t_llm:.2f}s + TTS:{t_tts:.2f}s)")
        print(f"  已对话 {state.dialog_count} 轮\n")
        
        stats.add_dialog(total_time)
        return True
        
    except Exception as e:
        print(f" 对话处理异常: {e}")
        import traceback
        traceback.print_exc()
        return False


def print_welcome():
    print("\n" + "="*70)
    print("  语音唤醒助手 v1.0")
    print("="*70)
    print("功能特性：")
    print("   语音唤醒 - 说出唤醒词激活助手")
    print("   连续对话 - 唤醒后可连续多轮对话")
    print("   智能控制 - 支持智能家居设备控制")
    print("   自动休眠 - 超时或达到上限自动休眠")
    print("")
    print("唤醒词列表：")
    for i, wake_word in enumerate(WAKE_WORDS[:5], 1):  # 只显示前5个
        print(f"  {i}. \"{wake_word}\"")
    if len(WAKE_WORDS) > 5:
        print(f"  ... 等共 {len(WAKE_WORDS)} 个唤醒词")
    print("")
    print("配置参数：")
    print(f"  对话超时: {DIALOG_TIMEOUT}秒")
    print(f"  最大连续对话: {MAX_DIALOG_TURNS}轮")
    print("")
    print("操作说明：")
    print("  - 说出唤醒词激活助手")
    print("  - 唤醒后直接说话即可对话")
    print("  - 说\"退下\"或\"休息吧\"让助手休眠")
    print("  - 按 Ctrl+C 退出程序")
    print("="*70 + "\n")


def main():
    """主函数"""
    print_welcome()
    
    state = DialogState()
    stats = PerformanceStats()
    
    try:
        # 播放启动提示音
        startup_text = "语音助手已启动，请说出唤醒词激活我"
        print(f" {startup_text}")
        try:
            startup_audio = tts_model.ort_predict(startup_text)
            play_audio(startup_audio)
        except:
            pass
        print()
        
        # 主循环
        while True:
            if not state.is_awake:
                print(" [休眠中] 监听唤醒词...", end='\r')
                
                wake_start = time.time()
                if listen_for_wake_word(state):
                    wake_time = time.time() - wake_start
                    stats.add_wake(wake_time)
                    
                    state.wake_up()
                    
                    wake_response = "我在，请说"
                    print(f" 助手: {wake_response}")
                    try:
                        wake_audio = tts_model.ort_predict(wake_response)
                        play_audio(wake_audio)
                    except:
                        pass
                    print()
                
                # 短暂延迟，避免CPU占用过高
                time.sleep(0.1)
            
            else:
                
                if state.should_sleep():
                    reason = "超时" if time.time() - state.last_interaction_time > DIALOG_TIMEOUT else "达到上限"
                    print(f" {reason}，自动进入休眠")
                    
                    sleep_text = SLEEP_PHRASES[1]
                    print(f" 助手: {sleep_text}")
                    try:
                        sleep_audio = tts_model.ort_predict(sleep_text)
                        play_audio(sleep_audio)
                    except:
                        pass
                    
                    state.sleep()
                    continue
                
                remaining_time = int(DIALOG_TIMEOUT - (time.time() - state.last_interaction_time))
                print(f" [唤醒中] 等待你的指令...（{remaining_time}秒后自动休眠）")
                
                success = process_dialog_turn(state, stats)
                
                if not success:
                    time.sleep(0.5)
    
    except KeyboardInterrupt:
        print("\n\n" + "="*70)
        print(" 程序被用户中断")
        print("="*70)
    
    except Exception as e:
        print(f"\n❌ 发生异常: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # 显示统计
        stats.print_summary()
        
        print("\n 清理资源...")
        print("感谢使用语音唤醒助手！再见！👋\n")


if __name__ == '__main__':
    main()

