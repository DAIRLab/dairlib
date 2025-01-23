import numpy as np
from sklearn.neural_network import MLPRegressor
import pandas as pd
from sklearn.svm import OneClassSVM

from joblib import dump


X = np.loadtxt('output.csv', delimiter=',')

# y = np.zeros((X.shape[0], 3))
# y[:,0] = X[:,-1]


model = OneClassSVM(kernel='rbf', nu=0.05, gamma='scale')

# Train the model
model.fit(X)

# Predict on the test set
y_pred = model.predict(X)

print(y_pred)

# Save the trained model to a file
dump(model, 'densetact.joblib')

# mse = mean_squared_error(y_test, y_pred)
# print(f"Mean Squared Error: {mse:.4f}")