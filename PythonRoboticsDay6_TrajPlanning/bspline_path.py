import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as si

# parameter
N = 3  # B Spline order


def bspline_planning(x, y, sn):
    t = range(len(x))
    x_tup = si.splrep(t, x, k=N)
    y_tup = si.splrep(t, y, k=N)

    x_list = list(x_tup)
    xl = x.tolist()
    x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

    y_list = list(y_tup)
    yl = y.tolist()
    y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

    ipl_t = np.linspace(0.0, len(x) - 1, sn)
    rx = si.splev(ipl_t, x_list)
    ry = si.splev(ipl_t, y_list)

    return rx, ry


def main():
    print(__file__ + " start!!")
    # way points
    #x = np.array([0.0, 0.0, 0.0, -5.0, 5.0])
    #y = np.array([0.0, 5.0, 10.0, 10.0, 10.0])
    x = np.array([4.0, 2.0, 0.0, 0.0, 2.0, 4.0])
    y = np.array([0.0, 0.0, 1.5, 3.5, 5.0, 5.0])
    sn = 10  # sampling number

    rx, ry = bspline_planning(x, y, sn)

    # show results
    plt.plot(x, y, '-ob', label="Waypoints")
    plt.plot(rx, ry, '-og', label="First Order B-Spline path")
    for xy in zip(rx,ry):
        plt.annotate("({}, {})".format(round(xy[0]),round(xy[1])), xy)
        #print("[{}, {}],".format(xy[0],xy[1]))
    rx, ry = bspline_planning(rx, ry, sn)
    print(2*x)
    print(2*y)
    plt.plot(rx, ry, '-r', label="Second Order kB-Spline path")
    plt.grid(True)
    plt.legend()
    plt.axis("equal")
    plt.show()


if __name__ == '__main__':
    main()
