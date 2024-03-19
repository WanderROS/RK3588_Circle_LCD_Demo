import cv2
import os

emoji_states = os.listdir("emoji")
for emoji_state in emoji_states:
    print(emoji_state)

    emoji_state_path = os.path.join("emoji",emoji_state)
    mp4s = os.listdir(emoji_state_path)
    print(mp4s)
    for mp4 in mp4s:
        mp4_path = os.path.join(emoji_state_path, mp4)

        if ".mp4" not in mp4_path:
            continue

        if "演示" in mp4_path:
            continue

        out_path = mp4_path[:-4].replace("emoji","image")
        os.makedirs(out_path,exist_ok=True)

        cap = cv2.VideoCapture(mp4_path)
        cnt = 0
        while True:
            ret, img = cap.read()
            if not ret:
                print(mp4_path,"视频结束")
                break

            cnt+=1
            img = cv2.resize(img, (240, 240))
            img_name = os.path.join(out_path,"%d.jpg"%(cnt))
            cv2.imwrite(img_name,img)

