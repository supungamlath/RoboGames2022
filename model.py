import csv
from pprint import pprint
import random
from skmultilearn.problem_transform import ClassifierChain
from sklearn.ensemble import RandomForestClassifier

from sklearn.model_selection import train_test_split
import sklearn.metrics as metrics
import time, pandas, ast
import numpy as np

class Model:
    def __init__(self):
        self.x_train = None
        self.x_test = None
        self.y_train = None
        self.y_test = None
        self.features = ["front-right","right-corner","right","left","left-corner","front-left"]
        self.labels=[(2, 2), (2, -2), (0, 3), (0, -3), (1, 3), (1, -3), (2, 3), (2, -3), (3, 1), (3, -1), (3, 0), (3, 2), (3, -2)]
        # self.labels=[(2, 2), (0, 3), (1, 3), (2, 3), (3, 1), (3, 0), (3, 2), (2, -2), (0, -3), (1, -3), (2, -3), (3, -1), (3, -2)]
        self.classifier = ClassifierChain(
            classifier = RandomForestClassifier(),
            require_dense = [False, True],
            order= list(range(len(self.labels)))
        )
        self.trainModel()
        self.testModel()

    def trainModel(self):
        print("Training Multi-label Classification Model")
        df = pandas.read_csv('training_dataset.csv')
        x = df[self.features]
        y = df[["blocked_cells"]]
        data = []
        for _, row in y.iterrows():
            b_cells = ast.literal_eval(row["blocked_cells"])
            c_row = []
            for c in self.labels:
                if c in b_cells:
                    c_row.append(1.0)
                else:
                    c_row.append(0.0)
            data.append(c_row)
        y = np.array(pandas.DataFrame(data, columns=self.labels))

        self.x_train, self.x_test, self.y_train, self.y_test = train_test_split(x ,y, random_state=100, test_size=0.2, shuffle=True)
        start=time.time()
        self.classifier.fit(self.x_train, self.y_train)
        print("Training time taken:'", round(time.time()-start,2),"seconds")

    def checkDataSet(self):
        df = pandas.read_csv('training_dataset.csv')
        # x = df[self.features]
        df["blocked_cells"] = df["blocked_cells"].apply(ast.literal_eval)
        y = df[["blocked_cells"]]
        data = []
        for _, row in y.iterrows():
            c_row = []
            for c in self.labels:
                if c in row["blocked_cells"]:
                    c_row.append(1)
                else:
                    c_row.append(0)
            data.append(c_row)
        y = pandas.DataFrame(data, columns=self.labels)
        counts = pandas.DataFrame(y.groupby(list(y.columns)).size(), columns=['count']).reset_index()
        print(counts)

    @staticmethod
    def generateSampleRecord(high_sensors, blocked_cells):
        sensor_names = ["front-right", "right-corner", "right", "rear-right", "rear-left", "left", "left-corner", "front-left"]

        with open("training_dataset.csv", "a+", newline="") as file:
            writer = csv.DictWriter(file, fieldnames = sensor_names + ["blocked_cells"])
            row = {}
            for sensor_name in sensor_names:
                if sensor_name in high_sensors:
                    row[sensor_name] = random.uniform(180.0, 390.0)
                else:
                    row[sensor_name] = random.uniform(60.0, 130.0)
            pprint(row)

            row["blocked_cells"] = blocked_cells
            writer.writerow(row)

    def testModel(self):
        y_hat = self.classifier.predict(self.x_test)
        # print(self.y_test)
        # print(y_hat.toarray())
        # diff = self.y_test - y_hat.toarray()
        # diff = pandas.DataFrame(diff, columns=self.labels)
        # comp = pandas.concat([self.x_test.reset_index(), diff], axis=1)
        # comp.to_excel('diffs.xlsx', index=False)

        cc_f1=metrics.f1_score(self.y_test, y_hat, average='micro')
        cc_hamm=metrics.hamming_loss(self.y_test,y_hat)
        print('Classifier Chain F1 Score:',round(cc_f1,3))
        print('Classifier Chain Hamming Loss:',round(cc_hamm,3))

    def predict(self, in_features):
        x_predict = []
        for feature in self.features:
            x_predict.append(in_features[feature])
        x_predict = np.array([x_predict])
        y_result = self.classifier.predict(x_predict)
        results = []
        for i, label in enumerate(self.labels):
            if y_result[0, i] == 1.0:
                results.append(label)
        return results

if __name__ == '__main__':
    model = Model()
    # Model.generateSampleRecord(["front-left", "front-right"], [(3, 1), (3, -1), (3, 0)])
    # Model.generateSampleRecord(["left", "right"], [(0, -3), (0, 3)])
    # Model.generateSampleRecord(["front-left", "front-right", "left-corner", "right-corner"], [(3, 1), (3, -1), (3, 0), (3, 2), (3, -2)])
    # Model.generateSampleRecord(["front-left", "front-right", "right-corner"], [(3, 1), (3, -1), (3, 0), (3, -2)])
    # Model.generateSampleRecord(["front-left", "front-right", "left-corner"], [(3, 1), (3, -1), (3, 0), (3, 2)])
    # Model.generateSampleRecord(["front-left", "front-right", "right-corner", "right"], [(3, 1), (3, -1), (3, 0), (3, -2), (0, -3), (1, -3), (2, -3)])
    # Model.generateSampleRecord(["front-left", "front-right", "left-corner", "left"], [(3, 2), (3, 1), (3, -1), (3, 0), (0, 3), (1, 3), (2, 3)])
    # Model.generateSampleRecord(["front-left", "front-right", "right"], [(3, 1), (3, -1), (3, 0), (0, -3)])
    # Model.generateSampleRecord(["front-left", "front-right", "left"], [(3, 1), (3, -1), (3, 0), (0, 3)])
    # Model.generateSampleRecord(["front-left", "front-right", "left-corner", "right-corner", "right"], [(3, 2), (3, 1), (3, -1), (3, 0), (3, -2), (0, -3), (1, -3), (2, -3)])
    # Model.generateSampleRecord(["front-left", "front-right", "left-corner", "right-corner", "left"], [(3, 2), (3, 1), (3, -1), (3, 0), (3, -2), (0, 3), (1, 3), (2, 3)])
    # Model.generateSampleRecord(["left-corner", "left"], [(2, 3), (1, 3), (0, 3)])
    # Model.generateSampleRecord(["right-corner", "right"], [(2, -3), (1, -3), (0, -3)])
    # Model.generateSampleRecord(["front-left"], [(3, 1)])
    # Model.generateSampleRecord(["front-right"], [(3, -1)])
    # Model.generateSampleRecord(["left-corner"], [(2, 2)])
    # Model.generateSampleRecord(["right-corner"], [(2, -2)])
    # Model.checkDataSet()
