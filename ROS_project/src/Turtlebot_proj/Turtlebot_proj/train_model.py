import os
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import OneHotEncoder
from sklearn.compose import ColumnTransformer
from sklearn.pipeline import Pipeline
from sklearn.ensemble import RandomForestClassifier
import joblib

def main():
    pkg_dir = os.path.dirname(__file__)
    csv_path = os.path.join(pkg_dir, "project_dataset.csv")
    print(f"Loading dataset from: {csv_path}")

    df = pd.read_csv(csv_path)

    feature_cols = ["Time of Day", "Task Type", "Room status"]
    target_col = "Target Room"

    X = df[feature_cols]
    y = df[target_col]

    categorical_cols = feature_cols
    preprocess = ColumnTransformer(
        transformers=[
            ("cat", OneHotEncoder(handle_unknown="ignore"), categorical_cols)
        ],
        remainder="drop",
    )
    rf_clf = RandomForestClassifier(
        n_estimators=200,
        random_state=42,
    )

    model = Pipeline(
        steps=[
            ("preprocess", preprocess),
            ("classifier", rf_clf),
        ]
    )

  
    X_train, X_test, y_train, y_test = train_test_split(
        X,
        y,
        test_size=0.2,
        random_state=42,
        stratify=y,
    )
   
    model.fit(X_train, y_train)

    train_acc = model.score(X_train, y_train)
    test_acc = model.score(X_test, y_test)

    print(f"Training accuracy: {train_acc:.4f}")
    print(f"Testing accuracy:  {test_acc:.4f}")

    model_path = os.path.join(pkg_dir, "room_decision_tree.pkl")
    joblib.dump(model, model_path)
    print(f"Model saved to: {model_path}")

if __name__ == "__main__":
    main()

