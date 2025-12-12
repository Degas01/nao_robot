import speech_recognition as sr
import time

def listen_and_write():
    r = sr.Recognizer()
    with sr.Microphone() as source:
        r.adjust_for_ambient_noise(source)
        print("Listening...")
        while True:
            try:
                audio = r.listen(source, timeout=3)
                text = r.recognize_google(audio,)
                print(f"Heard: {text}")
                with open("commands.txt", "a") as f:
                    f.write(text + "\n")
            except sr.WaitTimeoutError:
                continue
            except sr.UnknownValueError:
                print("Could not understand audio")
            except sr.RequestError as e:
                print(f"Error: {e}")
            except KeyboardInterrupt:
                print("Exiting speech recognition...")
                break

if __name__ == "__main__":
    listen_and_write()