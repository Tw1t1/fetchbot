import matplotlib
matplotlib.use('Agg')  # Use non-GUI backend

import cv2
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.pipeline import make_pipeline
from sklearn.ensemble import RandomForestRegressor, GradientBoostingClassifier
from sklearn.neural_network import MLPRegressor
from sklearn.metrics import mean_squared_error, r2_score, accuracy_score
from sklearn.model_selection import train_test_split
import joblib
import os
from datetime import datetime
from detection.process_image import BallDetector # change to your Ball detector


class BallCalibrator:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.distances = []
        self.sizes = []
        self.positions_x = []
        self.positions_y = []
        self.graspable = []  # New list to store graspable labels
        self.tuning_params = {
            "h_min": 103, "s_min": 50, "v_min": 50,
            "h_max": 110, "s_max": 255, "v_max": 255,
            "x_min": 0, "y_min": 0, "x_max": 100, "y_max": 100,
            "sz_min": 3, "sz_max": 255
        }
        self.detector = BallDetector(self.tuning_params, alpha=0.3)
        self.detector.create_tuning_window(self.tuning_params)

        self.min_distance = 2
        self.max_distance = 22
        self.distance_step = 2  # Smaller step for more granular data
        self.ball_diameter = 6.5  # adjust as needed
        self.output_dir = self.create_output_directory()

    def create_output_directory(self):
        current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = f"calibration_output_graspel_{current_time}"
        os.makedirs(output_dir, exist_ok=True)
        print(f"Output directory created: {output_dir}")
        return output_dir

    def collect_data(self):
        print(f"Starting data collection. Distance range: {self.min_distance}-{self.max_distance} cm, Step: {self.distance_step} cm")
        print("Measure the distance from the front edge of the ball (closest point to the camera) to the camera.")
        print("Move the ball to different positions in the frame for each distance.")
        print("Controls:")
        print("'c' - Capture data point")
        print("'g' - Toggle graspable (default is not graspable)")
        print("'n' - Next distance")
        print("'t' - Turn ON/OFF tuning windows")
        print("'q' - Quit")

        tuning_mode = False
        current_distance = self.min_distance
        is_graspable = False

        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            self.detector.update_params(self.detector.get_tuning_params())
            keypoints, out_image, mask = self.detector.find_circles(frame)

            keypoint = None
            kp_normalised = None
            if keypoints:
                keypoint = max(keypoints, key=lambda kp: kp.size)
                kp_normalised = self.detector.normalise_keypoint(out_image, keypoint)

            if tuning_mode:
                cv2.imshow('Tuning Output', out_image)
                cv2.imshow('Tuning Mask', mask)
            else:
                for window in ['Tuning Output', 'Tuning Mask']:
                    try:
                        cv2.destroyWindow(window)
                    except cv2.error:
                        pass
                if kp_normalised is not None:
                    frame = cv2.drawKeypoints(frame, [keypoint], np.array([]), (0, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                    cv2.putText(frame, f"Ball size: {kp_normalised.size:.2f}, X: {kp_normalised.pt[0]:.2f}, Y: {kp_normalised.pt[1]:.2f}", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(frame, f"Target distance: {current_distance} cm", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(frame, "Measure from front edge", (10, 90),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(frame, f"Graspable: {'Yes' if is_graspable else 'No'}", (10, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                cv2.imshow('Frame', frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('c'):
                if kp_normalised is not None:
                    self.distances.append(current_distance)
                    self.sizes.append(kp_normalised.size)
                    self.positions_x.append(kp_normalised.pt[0])
                    self.positions_y.append(kp_normalised.pt[1])
                    self.graspable.append(int(is_graspable))
                    print(f"Data point captured: Distance = {current_distance} cm, Size = {kp_normalised.size:.2f}, X = {kp_normalised.pt[0]:.2f}, Y = {kp_normalised.pt[1]:.2f}, Graspable = {is_graspable}")
                    print(f"Total data points: {len(self.distances)}")
                else:
                    print("No ball detected. Please try again.")
            elif key == ord('g'):
                is_graspable = not is_graspable
                print(f"Graspable toggled to: {is_graspable}")
            elif key == ord('n'):
                current_distance += self.distance_step
                if current_distance > self.max_distance:
                    print("Calibration complete!")
                    break
                print(f"Please move the ball to {current_distance} cm (measured from front edge) and press 'c' to capture points at different positions.")
            elif key == ord('t'):
                tuning_mode = not tuning_mode
                print(f"Tuning mode: {'On' if tuning_mode else 'Off'}")
            elif key == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

    def fit_models(self):
        if len(self.sizes) < 2:
            print("Not enough data points to fit models. Please collect at least 2 data points.")
            return None

        X = np.column_stack((self.sizes, self.positions_x, self.positions_y))
        y_distance = np.array(self.distances)
        y_graspable = np.array(self.graspable)

        X_train, X_test, y_distance_train, y_distance_test, y_graspable_train, y_graspable_test = train_test_split(
            X, y_distance, y_graspable, test_size=0.2, random_state=42)

        models = {
            'Linear': make_pipeline(LinearRegression()),
            'Polynomial': make_pipeline(PolynomialFeatures(2), LinearRegression()),
            'Random Forest': RandomForestRegressor(n_estimators=100, random_state=42),
            'Neural Network': MLPRegressor(hidden_layer_sizes=(10, 5), max_iter=2000, random_state=42)
        }

        for name, model in models.items():
            model.fit(X_train, y_distance_train)

        # Train the graspable classifier
        graspable_classifier = GradientBoostingClassifier(n_estimators=100, random_state=42)
        graspable_classifier.fit(X_train, y_graspable_train)

        # Evaluate models
        for name, model in models.items():
            y_pred = model.predict(X_test)
            mse = mean_squared_error(y_distance_test, y_pred)
            r2 = r2_score(y_distance_test, y_pred)
            print(f"{name} - MSE: {mse:.4f}, R2: {r2:.4f}")

        # Evaluate graspable classifier
        y_graspable_pred = graspable_classifier.predict(X_test)
        accuracy = accuracy_score(y_graspable_test, y_graspable_pred)
        print(f"Graspable Classifier Accuracy: {accuracy:.4f}")

        return models, graspable_classifier
    
    def dynamic_threshold(self, X):
        # This is a simple example of a dynamic threshold.
        # You may want to adjust this based on your specific requirements.
        size, x, y = X
        
        # Adjust threshold based on position
        if x < 0.3 or x > 0.7 or y < 0.3 or y > 0.7:
            return 30  # Harder to grasp at edges
        else:
            return 40  # Easier to grasp in the center

    def plot_results(self, models, graspable_classifier):
        X = np.column_stack((self.sizes, self.positions_x, self.positions_y))
        y = np.array(self.distances)

        plt.figure(figsize=(15, 10))

        for i, (x_pos, y_pos) in enumerate([(0.25, 0.25), (0.5, 0.5), (0.75, 0.75)]):
            plt.subplot(2, 2, i+1)
            sizes = np.linspace(X[:, 0].min(), X[:, 0].max(), 100)
            X_plot = np.column_stack((sizes, np.full(100, x_pos), np.full(100, y_pos)))

            for name, model in models.items():
                y_plot = model.predict(X_plot)
                plt.plot(sizes, y_plot, label=f'{name} model')

            # Plot dynamic threshold
            dynamic_thresholds = [self.dynamic_threshold((size, x_pos, y_pos)) for size in sizes]
            plt.plot(sizes, dynamic_thresholds, 'k--', label='Dynamic Threshold')

            # Plot graspable predictions
            graspable_pred = graspable_classifier.predict(X_plot)
            plt.scatter(sizes[graspable_pred == 1], np.full(np.sum(graspable_pred), 30), 
                        color='green', marker='^', s=100, label='Predicted Graspable')

            plt.scatter(X[:, 0], y, c=self.graspable, cmap='coolwarm', label='Data points')
            plt.xlabel('Apparent ball size')
            plt.ylabel('Distance (cm)')
            plt.title(f'Size vs Distance at position ({x_pos:.2f}, {y_pos:.2f})')
            plt.legend()
            plt.grid(True)

        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, 'calibration_models_comparison.png'))
        plt.close()

    def save_models(self, models, graspable_classifier):
        for name, model in models.items():
            filename = f'calibration_model_{name.lower().replace(" ", "_")}.joblib'
            filepath = os.path.join(self.output_dir, filename)
            joblib.dump(model, filepath)
            print(f"{name} model saved as {filepath}")

        # Save the graspable classifier
        filename = 'graspable_classifier.joblib'
        filepath = os.path.join(self.output_dir, filename)
        joblib.dump(graspable_classifier, filepath)
        print(f"Graspable classifier saved as {filepath}")

    def save_raw_data(self):
        filepath = os.path.join(self.output_dir, 'raw_calibration_data.csv')
        with open(filepath, 'w') as f:
            f.write("Distance,Size,X,Y,Graspable\n")
            for distance, size, x, y, graspable in zip(self.distances, self.sizes, self.positions_x, self.positions_y, self.graspable):
                f.write(f"{distance},{size},{x},{y},{graspable}\n")
        print(f"Raw calibration data saved to {filepath}")


if __name__ == "__main__":
    calibrator = BallCalibrator()
    calibrator.collect_data()

    models, graspable_classifier = calibrator.fit_models()
    if models:
        calibrator.plot_results(models, graspable_classifier)
        calibrator.save_models(models, graspable_classifier)
        calibrator.save_raw_data()

        # Example of how to use the calibrated models
        for size in range(1,11):
            real_size = size/10.0
            print(f"\nPredictions for ball size of {real_size:.1f} at position (0.5, 0.5)")
            X = np.array([[real_size, 0.5, 0.5]])
            for name, model in models.items():
                estimated_distance = model.predict(X)
                print(f"{name} model: estimated distance is {estimated_distance[0]:.2f} cm")

            graspable_pred = graspable_classifier.predict(X)
            print(f"Graspable: {'Yes' if graspable_pred[0] == 1 else 'No'}")

            dynamic_thresh = calibrator.dynamic_threshold((real_size, 0.5, 0.5))
            print(f"Dynamic threshold: {dynamic_thresh:.2f} cm")

        print(f"\nAll output files have been saved in the directory: {calibrator.output_dir}")
    else:
        print("Calibration failed. Please try again with more data points.")