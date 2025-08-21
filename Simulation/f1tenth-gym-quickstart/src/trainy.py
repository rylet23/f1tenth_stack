import numpy as np
from sklearn.ensemble import RandomForestRegressor
from sklearn.model_selection import train_test_split
import pandas as pd
import csv
import joblib
# Load and preprocess data
data = pd.read_csv('drive_data.csv')
data['prev_speed'] = data['speed'].shift(1)
data['prev_steering'] = data['steering_angle'].shift(1)
data.dropna(inplace=True)

x = data[['min_front_dist','prev_speed','prev_steering']]
y = data[['speed', 'steering_angle']]

X_train, X_test, y_train, y_test = train_test_split(x, y, test_size=0.2, random_state=42)

model = RandomForestRegressor()
model.fit(X_train, y_train)

preds = model.predict(X_test)

with open('prediction_data.csv', mode='w', newline='') as log_file:
    log_it = csv.writer(log_file)
    log_it.writerow(['speed', 'steering_angle'])
    for pred in preds:
        log_it.writerow(pred)
joblib.dump(model, 'my_rf_model.pkl')