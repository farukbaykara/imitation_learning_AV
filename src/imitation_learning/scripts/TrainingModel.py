print('Setting UP')
import os

from utlis import *
from sklearn.model_selection import train_test_split

#########1

path = '/home/aesk/baykara_ws/myData/'
data, data_steer = importDataInfo(path)

#########2

data = balanceData(data, data_steer, display=True)

#########3

imagesPath, steerings = loadData(path, data)
#print(imagesPath[0], steering[0])

#########4

xTrain, xVal, yTrain, yVal = train_test_split(imagesPath, steerings, test_size=0.2, random_state=5)
print('Total Training Images: ', len(xTrain))
print('Total Validation Images: ', len(xVal))

#5
#6
#7
#8

model = createModel()
model.summary()

#9
history = model.fit(batchGen(xTrain, yTrain, 100, 1), steps_per_epoch=20, epochs=3, validation_data = batchGen(xVal, yVal, 100, 0), validation_steps=200)

model.save('model.h5')
print('Model Saved')

plt.plot(history.history['loss'])
plt.plot(history.history['val_loss'])
plt.legend(['Training', 'Validation'])
plt.title('Loss')
plt.xlabel('Epoch')
plt.show()
