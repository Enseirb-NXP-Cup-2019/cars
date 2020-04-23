import matplotlib.pyplot as plt
import tkinter
import numpy as np

root = tkinter.Tk()
canvas = tkinter.Canvas(root)
canvas.pack()

points_initals = [[50, 15], [55, 0], [70, 15], [64, 0]]
points_equiv = [[5, 65], [5, 0], [60, 65], [60, 0]]

canvas.create_line(points_initals[0][0], points_initals[0][1], points_initals[1][0], points_initals[1][1], fill="red", width=1)

canvas.create_line(points_initals[2][0], points_initals[2][1], points_initals[3][0], points_initals[3][1], fill="red", width=1)

canvas.create_line(points_equiv[0][0], points_equiv[0][1], points_equiv[1][0], points_equiv[1][1], fill="blue", width=1)

canvas.create_line(points_equiv[2][0], points_equiv[2][1], points_equiv[3][0], points_equiv[3][1], fill="blue", width=1)


def dlt(p, p_ligne):
    A = np.zeros((8, 9), dtype=np.float64)
    for i in range(len(p)):
        x = p[i][0]
        y = p[i][1]
        u = p_ligne[i][0]
        v = p_ligne[i][1]
        A[2*i] = [-x, -y, -1, 0, 0, 0, u*x, u*y, u]
        A[2*i + 1] = [0, 0, 0, -x, -y, -1, v*x, v*y, v]

    print(A)
    u, s, vh = np.linalg.svd(A)
    H = vh[-1, :] / vh[-1, -1]
    return H.reshape(3, 3)


def apply_dlt(points, H):
    output = []
    for point in points:
        point_hom_coord = np.array([point[0], point[1], 1])
        r = np.matmul(H, point_hom_coord.T)
        r = r/r[2]
        output.append(np.array([round(r[0], 3), round(r[1], 3)]))
    return output


H = dlt(points_initals, points_equiv)

points_curve = [[55, 15], [55, 0], [70, 15], [60, 0]]

result_curve = apply_dlt(points_curve, H)

canvas.create_line(points_curve[0][0], points_curve[0][1], points_curve[1][0], points_curve[1][1], fill="green", width=1)
canvas.create_line(points_curve[2][0], points_curve[2][1], points_curve[3][0], points_curve[3][1], fill="green", width=1)

canvas.create_line(result_curve[0][0], result_curve[0][1], result_curve[1][0], result_curve[1][1], fill="yellow", width=1)
canvas.create_line(result_curve[2][0], result_curve[2][1], result_curve[3][0], result_curve[3][1], fill="yellow", width=1)


root.mainloop()
