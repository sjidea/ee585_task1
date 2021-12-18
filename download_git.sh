clear
rm -rf CubicSpline/ ee585_task1/
rm frenet_optimal_trajectory.py quintic_polynomials_planner.py #CubicSpline/cubic_spline_planner.py
rm frenet_optimal_trajectory.pyc quintic_polynomials_planner.pyc #CubicSpline/cubic_spline_planner.pyc
git clone https://github.com/sjidea/ee585_task1.git
mv ee585_task1/* .
rm output.txt