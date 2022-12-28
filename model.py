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
        self.labels=[(2, 2), (2, -2), (0, 3), (0, -3), (3, 1), (3, -1), (3, 0), (1, 3), (1, -3), (3, 2), (3, -2), (2, 3), (2, -3)]
        self.classifier = ClassifierChain(
            classifier = RandomForestClassifier(),
            require_dense = [False, True],
            order=[i for i in range(len(self.labels))]
        )
        self.trainModel()
        self.testModel()

    def trainModel(self):
        df = pandas.read_csv('training_dataset.csv')
        x = df[self.features]
        y = df[["blocked_cells"]]
        data = []
        for index, row in y.iterrows():
            b_cells = ast.literal_eval(row['blocked_cells'])
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
        print('Training time taken:', round(time.time()-start,2),'seconds')

    def testModel(self):
        y_hat = self.classifier.predict(self.x_test)
        print(self.y_test)
        print(y_hat.toarray())
        cc_f1=metrics.f1_score(self.y_test, y_hat, average='micro')
        cc_hamm=metrics.hamming_loss(self.y_test,y_hat)
        print('Classifier Chain F1-score:',round(cc_f1,3))
        print('Classifier Chain Hamming Loss:',round(cc_hamm,3))

    def predict(self, in_features):
        x_predict = []
        for feature in self.features:
            x_predict.append(in_features[feature])
        x_predict = np.array([x_predict])
        y_result = self.classifier.predict(x_predict)
        results = []
        for i in range(len(self.labels)):
            if y_result[0, i] == 1.0:
                results.append(self.labels[i])
        return results

if __name__ == '__main__':
    model = Model()
