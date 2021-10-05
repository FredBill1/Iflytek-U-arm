import re
import requests
from urllib import error
from bs4 import BeautifulSoup
import os
 
num = 0
numPicture = 0
file = ''
List = []
 
def Find(url, Headers):
    global List
    print('正在检测图片总数，请稍等.....')
    t = 0
    i = 1
    s = 0
    while t < 1000:
        Url = url + str(t)
        try:
            Result = requests.get(Url, headers=Headers,timeout=7)#超过7s连不上则不再连
        except BaseException:
            t = t + 60
            continue
        else:
            result = Result.text
            pic_url = re.findall('"objURL":"(.*?)",', result, re.S)  # 先利用正则表达式找到图片url
            s += len(pic_url)
            if len(pic_url) == 0:
                break
            else:
                List.append(pic_url)
                t = t + 60
    return s
 
# 推荐类似图片
# def recommend(url):
    Re = []
    try:
        html = requests.get(url)
    except error.HTTPError as e:
        return
    else:
        html.encoding = 'utf-8'
        bsObj = BeautifulSoup(html.text, 'html.parser')
        div = bsObj.find('div', id='topRS')
        if div is not None:
            listA = div.findAll('a')
            for i in listA:
                if i is not None:
                    Re.append(i.get_text())
        return Re
 
 
def dowmloadPicture(html, keyword, file1, Headers):
    global num
    # t =0
    pic_url = re.findall('"objURL":"(.*?)",', html, re.S)  # 先利用正则表达式找到图片url
    print('找到关键词:' + keyword + '的图片，即将开始下载图片...')
    for each in pic_url:
        print('正在下载第' + str(num + 1) + '张图片，图片地址:' + str(each))
        try:
            if each is not None:
                pic = requests.get(each, headers=Headers, timeout=7)
            else:
                continue
        except BaseException:
            print('错误，当前图片无法下载')
            continue
        else:
            string = './'+file1+'/'+keyword + '_' + str(num) + '.jpg'
            fp = open(string, 'wb')
            fp.write(pic.content)
            fp.close()
            num += 1
        if num >= numPicture:
            return
 
 
if __name__ == '__main__':  # 主函数入口
    word = input("请输入搜索关键词: ")
    url = 'http://image.baidu.com/search/flip?tn=baiduimage&ie=utf-8&word=' + word + '&pn='
    Headers={"User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebkit/537.36 (HTML, like Gecko) Chrome/71.0.3578.98 Safari/537.36"}
    tot = Find(url, Headers)
    #Recommend = recommend(url)  # 记录相关推荐
    print('经过检测%s类图片共有%d张' % (word, tot))
    numPicture = int(input('请输入想要下载的图片数量 '))
    file = input('请建立一个存储图片的文件夹，输入文件夹名称即可')
    y = os.path.exists(file)
    if y == 1:
        print('该文件已存在，请重新输入')
        file = input('请建立一个存储图片的文件夹，输入文件夹名称即可')
        os.mkdir(file)
    else:
        os.mkdir(file)
    t = 0
    tmp = url
    while t < numPicture:
        try:
            url = tmp + str(t)
            result = requests.get(url, headers=Headers, timeout=10)
            print(url)
        except error.HTTPError as e:
            print('网络错误，请调整网络后重试')
            t = t+60
        else:
            dowmloadPicture(result.text, word, file, Headers)
            t = t + 60
 
    print('当前搜索结束，感谢使用')

    # 推荐其他类似图片  
    # print('猜你喜欢')
    # for re in Recommend:
    #     print(re, end='  ')
    
