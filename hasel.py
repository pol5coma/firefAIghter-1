import numpy as np
import threading
import multiprocessing
import math

def rgb2hsl(rgb):

    def core(_rgb, _hsl):

        irgb = _rgb.astype(np.uint16)
        ir, ig, ib = irgb[:, :, 0], irgb[:, :, 1], irgb[:, :, 2]
        h, s, l = _hsl[:, :, 0], _hsl[:, :, 1], _hsl[:, :, 2]

        imin, imax = irgb.min(2), irgb.max(2)
        iadd, isub = imax + imin, imax - imin

        ltop = (iadd != 510) * (iadd > 255)
        lbot = (iadd != 0) * (ltop == False)

        l[:] = iadd.astype(np.float) / 510

        fsub = isub.astype(np.float)
        s[ltop] = fsub[ltop] / (510 - iadd[ltop])
        s[lbot] = fsub[lbot] / iadd[lbot]

        not_same = imax != imin
        is_b_max = not_same * (imax == ib)
        not_same_not_b_max = not_same * (is_b_max == False)
        is_g_max = not_same_not_b_max * (imax == ig)
        is_r_max = not_same_not_b_max * (is_g_max == False) * (imax == ir)

        h[is_r_max] = ((0. + ig[is_r_max] - ib[is_r_max]) / isub[is_r_max])
        h[is_g_max] = ((0. + ib[is_g_max] - ir[is_g_max]) / isub[is_g_max]) + 2
        h[is_b_max] = ((0. + ir[is_b_max] - ig[is_b_max]) / isub[is_b_max]) + 4
        h[h < 0] += 6
        h[:] /= 6

    hsl = np.zeros(rgb.shape, dtype=np.uint16)
    cpus = multiprocessing.cpu_count()
    length = int(math.ceil(float(hsl.shape[0]) / cpus))
    line = 0
    threads = []
    while line < hsl.shape[0]:
        line_next = line + length
        thread = threading.Thread(target=core, args=(rgb[line:line_next], hsl[line:line_next]))
        thread.start()
        threads.append(thread)
        line = line_next

    for thread in threads:
        thread.join()

    return hsl