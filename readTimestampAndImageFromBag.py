from rosbag2_api import read_bag
import cv2
# from sensor_msgs import msg
from cv_bridge import CvBridge

# cam0: left camera id = 1
# cam1: right camera id = 2

bag_path_1 = "bags/rosbag2_2023_06_25-14_02_30/rosbag2_2023_06_25-14_02_30_0.db3"
bag_path_2 = "bags/rosbag2_2023_06_25-14_02_31/rosbag2_2023_06_25-14_02_31_0.db3"
pathList = [bag_path_1,bag_path_2]
topic_name_1 = "/my_camera_1/pylon_ros2_camera_node/image_raw/compressed"
topic_name_2 = "/my_camera_2/pylon_ros2_camera_node/image_raw/compressed"
topicList = [topic_name_1,topic_name_2]
save_path = "/data/readbag/images/multiCAM_cali_8/"

class parseCompressedImage():
    """
    bag_path : The path of .db3 files.
    topic_name : The name of topic where you want to extract from.
    print_out : Print some information about results.
    """
    def __init__(self, bag_path:str, topic_name:str, save_path:str = None, print_out:bool=False):
        self.bridge = CvBridge()
        self.bag_path = bag_path
        self.topic_name = topic_name
        self.print_out = print_out
        self.save_path = save_path
       
    def getTnM(self):
        tstmp,msgs = read_bag.read_from_topic(bag_file   = self.bag_path,
                                              topic_name = self.topic_name,
                                              print_out  = self.print_out)
        return (tstmp,msgs)


    def extractImage(self):
        # Get timestamps and messages.
        tstmp,msgs = self.getTnM()
        if self.print_out:
            print("Number of timestamps: "+len(tstmp))
            print("Number of messages: "+len(msgs))

        # Save images with the name of their timestamps.
        if not self.save_path == None:
            for i in range(len(msgs)):
                img = self.bridge.compressed_imgmsg_to_cv2(cmprs_img_msg=msgs[i])
                # cv2.imshow(str(tstmp[i]),img)
                # cv2.waitKey(10)
                # cv2.destroyWindow(str(tstmp[i]))
                name=self.save_path+str(tstmp[i])+".png"
                cv2.imwrite(name,img)

class alignTmstmp():
    """
    paths : A list of .db3 file paths of cameras.
    topic : A list of topic names.
    """
    def __init__(self, paths:list, topics:list):
        self.paths = paths
        self.topics = topics

    def cmpTimestamp(self, t1:int, t2:int, threshold:float = 0.01):
        if abs((t1-t2)/1e9) < threshold:
            return True
        else:
            return False


    def align(self, stmpList1:list, stmpList2:list, msgs1:list, msgs2:list):
        newList1 = []
        newList2 = []
        newMsgs1 = []
        newMsgs2 = []
        i = 0
        j = 0
        num = min(len(stmpList1),len(stmpList2))
        if stmpList1[0]>stmpList2[0]:
            while not self.cmpTimestamp(stmpList1[i],stmpList2[j]):
                j = j + 1
                if j == 200:# search in first 10 seconds
                    j = 0
                    i = i + 1
        else:
            while not self.cmpTimestamp(stmpList1[i],stmpList2[j]):
                i = i + 1
                if i == 200:# search in first 10 seconds
                    i = 0
                    j = j + 1
        ii = i
        jj = j  
        while ii<num:
                # Timstamps are aligned originally.
                if self.cmpTimestamp(stmpList1[ii],stmpList2[jj]):
                    newList1.append(stmpList1[ii])
                    newList2.append(stmpList2[jj])
                    newMsgs1.append(msgs1[ii])
                    newMsgs2.append(msgs2[jj])
                    ii = ii + 1
                    jj = jj + 1
                    continue
                # Timestamps only have 1 frame difference.
                else:
                    if self.cmpTimestamp(stmpList1[ii],stmpList2[jj+1]):
                        jj = jj + 1
                        newList1.append(stmpList1[ii])
                        newList2.append(stmpList2[jj])
                        newMsgs1.append(msgs1[ii])
                        newMsgs2.append(msgs2[jj])
                        ii = ii + 1
                        jj = jj + 1
                        continue
                    elif self.cmpTimestamp(stmpList1[ii+1],stmpList2[jj]):
                        ii = ii + 1
                        newList1.append(stmpList1[ii])
                        newList2.append(stmpList2[jj])
                        newMsgs1.append(msgs1[ii])
                        newMsgs2.append(msgs2[jj])
                        ii = ii + 1
                        jj = jj + 1
                        continue
                    elif self.cmpTimestamp(stmpList1[ii+1],stmpList2[jj+1]):
                        ii = ii + 1
                        jj = jj + 1
                        newList1.append(stmpList1[ii])
                        newList2.append(stmpList2[jj])
                        newMsgs1.append(msgs1[ii])
                        newMsgs2.append(msgs2[jj])
                        ii = ii + 1
                        jj = jj + 1
                        continue
                    # More than one frame lost.
                    else:
                        print("Notice that more than one frame lost around timestamp",ii,jj,".")
                        iitmp = ii
                        jjtmp = jj
                        while not self.cmpTimestamp(stmpList1[iitmp],stmpList2[jjtmp]):
                            jjtmp = jjtmp + 1
                            if jjtmp == jj + 10:# search in the following 0.5 seconds
                                jjtmp = jj
                                break
                        if jjtmp == jj:
                            while not self.cmpTimestamp(stmpList1[iitmp],stmpList2[jjtmp]):
                                iitmp = iitmp + 1
                                if iitmp == ii + 10:# search in the following 0.5 seconds
                                    iitmp = ii
                                    break
                        if jjtmp == jj and iitmp == ii:
                            print("The data pair were broke, Please check them manually.")
                            break
                        ii = iitmp
                        jj = jjtmp

                        
        return (newList1,newList2,newMsgs1,newMsgs2,i,j)


    def main(self):
        tstmpList = []
        msgsList = []
        newTstmpList = []
        newMsgsList = []
        for i in range(len(self.paths)):
            tstmp,msgs = parseCompressedImage(self.paths[i],self.topics[i]).getTnM()
            tstmpList.append(tstmp)
            msgsList.append(msgs)
        for i in range(len(tstmpList)-1):
            newStmp1,newStmp2,newMsgs1,newMsgs2,idx1,idx2 = self.align(tstmpList[0],tstmpList[i+1],msgsList[0],msgsList[i+1])
            print("The first aligned timestamp pair is:")
            print("Topic Name   |   Timestamp")
            print(self.topics[0], "|", idx1)
            print(self.topics[i+1], "|", idx2)
            if i == 0:
                newTstmpList.append(newStmp1)
                newMsgsList.append(newMsgs1)
            newTstmpList.append(newStmp2)
            newMsgsList.append(newMsgs2)
        return(newTstmpList, newMsgsList)
    
        
# #　Extract unaligned data
# parseCompressedImage(bag_path_1,topic_name_1).extractImage()


align = alignTmstmp(pathList,topicList)
TstmpsList,MsgsList = align.main()

with open("offset.txt","w") as f:
    for i in range(len(TstmpsList[0])):
        offset = (TstmpsList[0][i] - TstmpsList[1][i])/1e9
        f.write(str(offset)+"\n")
    f.close
for i in range(len(pathList)):
    filename = "cam"+ str(i) + "_timestamp.txt"
    with open(filename,"w") as f:
        for j in range(len(TstmpsList[i])):
            f.write(str(TstmpsList[i][j])+"\n")
    f.close
bridge = CvBridge()
for i in range(len(MsgsList)):
    for j in range(len(MsgsList[i])):
                    img = bridge.compressed_imgmsg_to_cv2(cmprs_img_msg=MsgsList[i][j])
                    name=save_path+"cam"+str(i)+"/"+str(TstmpsList[i][j])+".png"
                    cv2.imwrite(name,img)