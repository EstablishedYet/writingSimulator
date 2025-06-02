from aip import AipSpeech, AipOcr
import speech_recognition as sr
from playsound import playsound
import erniebot
import numpy as np
import cv2
from pydub import AudioSegment
from pydub.playback import play
import time
import io
import jkrc
from scipy.spatial.transform import Rotation as R

APP_ID = '118123134'
API_KEY = 'SQ4mbtEKuPf4ObydvOtRYNGy'
SECRET_KEY = 'lzl4opbY1hAs0PqV3D05ETcOAAMJRi7F'
client1 = AipSpeech(APP_ID, API_KEY, SECRET_KEY)

erniebot.api_type = "aistudio"
erniebot.access_token = "058d0199abffa98cf139dac355df269b9fa28b0f"

r = sr.Recognizer()
mic = sr.Microphone()

def pixel_to_world(u, v, fx, fy, cx, cy, Z):
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    return X, Y

def get_text(wav_bytes):
    text=client1.asr(wav_bytes, 'wav', 16000, {
    'dev_pid': 1537,
})
    return text

def get_file_content(filePath):
    with open(filePath, 'rb') as fp:
        return fp.read()
    
def get_file_string(filePath):
    with open(filePath, 'r', encoding='utf-8') as fp:
        return fp.read()
    
def initializing():
    global results
    results=np.full(4,-1)
    string1="请指定列数、行数、列距和行距。"
    result_1  = client1.synthesis(string1, 'zh', 1, {
                    'vol': 5,
                    })
    if not isinstance(result_1, dict):
        name="audio.mp3"
        with open(name, 'wb') as f:
            f.write(result_1)
        playsound(name)
    else:
        print("TTS failed")
    while True:
        try:
            with mic as source:
                r.adjust_for_ambient_noise(source)
                audio = r.listen(source) 
            audio_data = audio.get_wav_data(convert_rate=16000)
            text = get_text(audio_data)['result'][0]
            content = "我现在希望你充当为我从文本中提取信息的工具，你的回答只包含我所需，不得包含其余任何文字。请从以下文本中提取信息,识别其中的列数、行数、列距、行距信息。你的回答只包含一行四个数字,以空格作分隔符,第一个数为列数的阿拉伯数字,第二个数为行数的阿拉伯数字,第三个数为列距的阿拉伯数字,第四个数为行距的阿拉伯数字,若未识别到某信息则该处填充'X'。文本如下(若文本为空则输出'X X X X'):"
            content += text

            messages = [
                {
                    "role": "user",
                    "content": content,
                }
            ]

            response = erniebot.ChatCompletion.create(
                model="ernie-4.0",
                messages=messages,
            )
            result=response.result
            numbers=result.split()
            print(numbers)
            words=["列数","行数","列距","行距"]
            warning=""
            flag=True
            for i,number in enumerate(numbers):
                print(i)
                if i<=3:
                    if not number.isdigit() and results[i]==-1:
                        flag=False
                        warning+=words[i]
                        warning+=","
                    else:
                        if number.isdigit():
                            results[i]=number
                else:
                    break
            if(flag):
                break
            else:
                warning="请复述"+warning
                warning+="的取值"
                result_1  = client1.synthesis(warning, 'zh', 1, {
                    'vol': 5,
                    })
                print(result)
            audio_file = io.BytesIO(result_1)
            audio_segment = AudioSegment.from_file(audio_file, format="mp3")
            play(audio_segment)
        
        except Exception as e:
            print(e)
            print('Finished with error')
            break
    prompt="将以"+str(results[0])+"为列数,"+str(results[1])+"为行数，"+str(results[2])+"为列距，"+str(results[3])+"为行距书写"
    result_1  = client1.synthesis(prompt, 'zh', 1, {
                    'vol': 5,
                    })
    audio_file = io.BytesIO(result_1)

    audio_segment = AudioSegment.from_file(audio_file, format="mp3")
    play(audio_segment)

def load_traj_from_txt(txt_path):
    data = np.loadtxt(txt_path, delimiter=' ', dtype=np.float32)
    return data

def order_points_clockwise(pts):
    rect = np.zeros((4, 2), dtype="float32")

    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]   
    rect[2] = pts[np.argmax(s)]   

    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)] 
    rect[3] = pts[np.argmax(diff)]

    return rect

def writing():
    fx = 724.06977275
    fy = 722.61805227
    cx = 348.11775966
    cy = 183.19284565

    Z = 200
    
    string=get_file_string("example.txt")
    print(string)
    global rc
    rc = jkrc.RC("10.5.5.100")
    print(f"Login: {rc.login()}")
    print(f"Power on: {rc.power_on()}")
    print(f"Enable robot: {rc.enable_robot()}")
    rc.set_user_frame_id(2)
    print(rc.get_tcp_position())
    tcp_pos=[0,0,0,0,0,0]  

    ret=rc.linear_move(tcp_pos,0,True,20)  
    rc.set_user_frame_data(4, [0,0,0,0,0,0],"4")
    rc.set_user_frame_id(4)
    rc.set_user_frame_id(2)

    global ABS,INCR
    ABS = 0  
    INCR= 1  
    cap = cv2.VideoCapture(0)
    ret, img = cap.read()

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    try:
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
            if len(approx) == 4:
                corners = approx.reshape(4, 2)
                print(corners)
                corners=order_points_clockwise(corners)
                print(corners)
                break
    except UnboundLocalError as e:
        print(e)
        print("No paper found! Please adjust the camera!")

    c=np.zeros((4,2))
    d=np.zeros((3,2))

    for i,(u, v) in enumerate(corners):
        x, y = pixel_to_world(u, v, fx, fy, cx, cy, Z)
        c[i]=(x,y)
    print(c)

    xv=np.array([c[1][0]-c[0][0],c[1][1]-c[0][1],0])
    xvn=np.linalg.norm(xv)
    xv=xv/xvn
    yv=np.array([c[3][0]-c[0][0],c[3][1]-c[0][1],0])
    yvn=np.linalg.norm(yv)
    yv=yv/yvn
    zv=np.cross(xv,yv)
    R_mat = np.column_stack((xv, yv, zv))
    r = R.from_matrix(R_mat)
    rx, ry, rz = r.as_euler('xyz')
    user_coord = [c[0][0], c[0][1], 100, 0, 0, 0]

    user_coord=np.array(user_coord)
    print(user_coord)
    dx=(xvn-(results[0]+1)*results[2])/results[0]
    dy=(xvn-(results[1]+1)*results[3])/results[1]
    print(c)
    print(xvn)
    print(yvn)
    print(dx)
    print(dy)

    rc.linear_move(user_coord,0,True,20)
    rc.set_user_frame_data(1, [0,0,0,0,0,0],"1")
    rc.set_user_frame_id(1)
    present=rc.get_tcp_position()[1]
    for i in range(3,6):
        present[i]=round(present[i]/(0.7853975))*0.7853975
    print("present:")
    print(present)
    rc.linear_move(present,ABS,True,20)
    rc.set_user_frame_data(3, present,"3")
    rc.set_user_frame_id(3)
    print(rc.get_tcp_position())
    present=np.array(rc.get_tcp_position()[1])
    suffix=present[-3:]

    for i,ch in enumerate(string):
        print(i)
        split=singleChar(ch)
        for stroke in split:
            stroke[:,0]*=dx
            stroke[:,1]*=dy
            i0=i%results[0]
            stroke[:,0]+=i0*dx+(i0+1)*results[2]
            j=i/results[0]
            stroke[:,1]+=j*dy+(j+1)*results[3]
            spe=stroke[0].copy()
            spe[2]=-50
            result=np.hstack((spe,suffix))
            print(result)
            rc.linear_move(result,ABS,True,20)
            for j in stroke:
                j=np.hstack((j,suffix))
                print(j)
                rc.linear_move(j,ABS,True,20)


def singleChar(ch):
    str = "./results_one/" + ch
    coordinates = load_traj_from_txt(str)
    coordinates[:, 0] = np.cumsum(coordinates[:, 0])
    coordinates[:, 1] = np.cumsum(coordinates[:, 1])
    
    ids = np.where(coordinates[:, -1] == 1)[0] 
    if len(ids) < 1:
        ids = np.where(coordinates[:, 3] == 1)[0] + 1
        if len(ids) < 1:
            ids = np.array([len(coordinates)])
            xys_split = np.split(coordinates, ids, axis=0)[:-1]
        else:
            xys_split = np.split(coordinates, ids, axis=0)
    else:
        remove_end = np.split(coordinates, ids, axis=0)[0]
        print("remove_end")
        print(remove_end)
        ids = np.where(remove_end[:, 3] == 1)[0] + 1
        xys_split = np.split(remove_end, ids, axis=0)[:-1]
        print("xys")
        print(xys_split)

    min_x, max_x = float('inf'), float('-inf')
    min_y, max_y = float('inf'), float('-inf')

    for stroke in xys_split:
        for (x, y) in stroke[:, :2].reshape((-1, 2)):
            min_x = min(x, min_x)
            max_x = max(x, max_x)
            min_y = min(y, min_y)
            max_y = max(y, max_y)

    original_size = max(max_x - min_x, max_y - min_y)

    normalized_strokes = []
    for stroke in xys_split:
        stroke = stroke[:, :2]
        stroke[:, 0] -= min_x
        stroke[:, 1] -= min_y
        stroke /= original_size
        zs = np.full((stroke.shape[0], 1), 30)
        stroke = np.hstack((stroke, zs))
        normalized_strokes.append(stroke)
    print(normalized_strokes)
    return normalized_strokes


if __name__=="__main__":
    # initializing() #语言识别精度极低，故此处采取直接指定列数、行数、列距、行距的办法。
    results=[2,2,5,5] 
    writing()