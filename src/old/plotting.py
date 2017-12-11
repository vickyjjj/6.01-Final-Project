import numpy as np
import matplotlib.pyplot as plt


for i in range(1):
    three_fingers = 'three_finger_' + str(i) + '.txt'
    two_fingers = 'two_finger_' + str(i) + '.txt'
    one_finger = 'one_finger_' + str(i) + '.txt'
    data_1 = np.loadtxt(one_finger)
    data_1 = data_1.transpose()
    data_2 = np.loadtxt(two_fingers)
    data_2 = data_2.transpose()
    data_3 = np.loadtxt(three_fingers)
    data_3 = data_3.transpose()
    # plotting each data
##    for j in rang(8):
##        _1_finger = plt.plot(range(8), data_1[j][:], '.')
##        _2_finger = plt.plot(range(8), data_2[j][:], '*')
##        _3_finger = plt.plot(range(8), data_3[j][:], '-')
##        plt.legend((_1_finger, _2_finger, _3_finger), ('one finger', 'two fingers', 'three fingers'))
##        plt.title('row %d of three fingers'%j)
##        plt.show()
    number = 1
    for finger in [data_1, data_2, data_3]:
        plots = []
        for j in range(8):
            row = plt.plot(range(8), finger[j][:], label = 'row ' + str(j))
            plots.append(row)
        #plt.legend(handles=[plots])
        plt.title('Finger %d rows'%number)
        plt.show()
        number += 1
        
