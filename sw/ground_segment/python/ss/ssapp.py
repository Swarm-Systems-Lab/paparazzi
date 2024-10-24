import wx
import sys
from os import path, getenv

from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas
import matplotlib.pyplot as pl
import numpy as np
import matplotlib.colors as mcolors

DEF_PPRZ_HOME = path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../'))

sys.path.append(getenv("PAPARAZZI_HOME", DEF_PPRZ_HOME) + "/var/lib/python")
sys.path.append(getenv("PAPARAZZI_SRC", DEF_PPRZ_HOME) + "/sw/lib/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

WX_WIDTH = 800
WX_HEIGHT = 800

class SSDevice:
    def __init__(self, dw_addrs=[], sigma=0.0, centroid=np.array([0, 0]),
                asc_dir=np.array([0, 0]), pos=np.array([0, 0])):
        self.dw_addrs: list[int] = dw_addrs
        self.sigma: float = sigma
        self.centroid = centroid
        self.asc_dir = asc_dir
        self.pos = pos
        
class SSapp:
    def __init__(self):
        self.devices: dict[int, SSDevice] = {}
        
        # TODO Singleton
        self.ivy_interface = IvyMessagesInterface("SS")
        self.ivy_interface.subscribe(self.dw_data_recv, PprzMessage("telemetry", "DWM1001_DATA"))
        self.ivy_interface.subscribe(self.dw_debug_recv, PprzMessage("telemetry", "DWM1001_DEBUG"))
        
    def dw_data_recv(self, ac_id, msg):
        msg_dict = msg.to_dict()
        if ac_id in self.devices:
            self.devices[ac_id].sigma = float(msg_dict['sigma'])
            self.devices[ac_id].centroid = np.array([float(v) for v in msg_dict['centroid']])
            self.devices[ac_id].asc_dir = np.array([float(v) for v in msg_dict['asc_dirc']])
            self.devices[ac_id].pos = np.array([float(v) for v in msg_dict['enu_pos']])
        
    def dw_debug_recv(self, ac_id, msg):
        msg_dict = msg.to_dict()
        if ac_id not in self.devices:
            self.devices[ac_id] = SSDevice(dw_addrs=[int(addr) for addr in msg_dict['nei_addresses']])
            print(f'\n {ac_id} DEVICE CREATED\n')

    def to_np_arrays(self):
        npa = {}
        if self.devices:
            npa['sigma'] = np.array([value.sigma for value in self.devices.values()])
            npa['pos'] = np.vstack([np.array(value.pos) for value in self.devices.values()])
            npa['centroid'] = np.vstack([np.array(value.centroid) for value in self.devices.values()])
            npa['asc_dir'] = np.vstack([np.array(value.asc_dir) for value in self.devices.values()])
            
            comp_cen = np.average(npa['pos'], axis=0)
            comp_asc_dir = npa['sigma'][:,np.newaxis]*npa['centroid']
            comp_asc_dir = np.average(comp_asc_dir, axis=0)
            comp_asc_dir = comp_asc_dir/np.linalg.norm(comp_asc_dir, axis=0, keepdims=True)
            comp_asc_dir = comp_asc_dir[np.newaxis, :]
            comp_asc_dir = np.broadcast_to(comp_asc_dir, npa['asc_dir'].shape)
            
            npa['centroid'] = npa['pos']- npa['centroid']
            npa['comp_centroid'] = comp_cen
            npa['comp_asc_dir'] = comp_asc_dir
        return npa
    
    def __del__(self):
        self.ivy_interface.shutdown()

class SSFrame(wx.Frame):
    def __init__(self):
        super().__init__(id=-1, parent=None, name=u'SS', 
                        size=wx.Size(WX_WIDTH, WX_HEIGHT),
                        style=wx.DEFAULT_FRAME_STYLE, title=u'Source Seeking')
        
        self.ssapp = SSapp()
        self.fig = pl.figure(figsize=(WX_WIDTH/100, WX_HEIGHT/100))
        
        self.canvas = FigureCanvas(self, -1, self.fig)
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        self.redraw_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.OnRedrawTimer, self.redraw_timer)
        self.redraw_timer.Start(100)

    def draw_ss(self):
        self.fig.clear()
        ax = self.fig.subplots()
        
        ax.set_aspect("equal")
        ax.grid(True)

        title = r"SS"
        ax.set_title(title)

        ax.set_xlabel("$Y$ [L]")
        ax.set_ylabel("$X$ [L]")

        # Lines
        ax.axhline(0, c="k", ls="-", lw=1.1)
        ax.axvline(0, c="k", ls="-", lw=1.1)
        ax.set_box_aspect(1)
        ax.margins(0.25, 0.25)
        
        data = self.ssapp.to_np_arrays()
        
        # data = {
        # 	'sigma' : np.array([0,0,0]),
        # 	'centroid' : np.zeros((3,2)),
        # 	'pos' : np.zeros((3,2)),
        # 	'asc_dir' : np.zeros((3,2)),
        # 	'comp_centroid' : np.zeros((3,2)),
        # 	'comp_asc_dir' : np.zeros((3,2))
        # }
  
        if data:
            self.fig.tight_layout()
            p_max = np.array([np.max(data['pos'][:,0]),np.max(data['pos'][:,1])])
            p_min = np.array([np.min(data['pos'][:,0]),np.min(data['pos'][:,1])])

            scale = np.max(np.linalg.norm(p_max-p_min))
            ds = scale/4
            
            s_max = np.max(data['sigma'])
            if s_max > 1e-5:
                s_min = np.min(data['sigma'][data['sigma']>1e-5])
            else:
                s_min = 0
            
            # Set up the normalization and colormap
            color_norm = mcolors.Normalize(vmin=s_min, vmax=s_max)
            cmap = pl.cm.get_cmap('RdYlGn_r')  # 'RdYlGn_r' is green to red
        
            # Agents
            ax.scatter(data['pos'][:,0], data['pos'][:,1], c=data['sigma'], s=100, cmap=cmap, norm=color_norm)
            
            # Centroid (true consensus adjusted centroid)
            ax.scatter(data['centroid'][:,0], data['centroid'][:,1], color="r", marker=r"$x$", s=50)
            
            # Asc_dir
            ax.quiver(data['pos'][:,0], data['pos'][:,1], data['asc_dir'][:,0], data['asc_dir'][:,1], color="r", angles='xy', scale_units='xy', scale=1/ds)
            
            # Comp centroid
            ax.scatter(data['comp_centroid'][0], data['comp_centroid'][1], color="b", marker=r"$x$", s=50)
            
            # Comp asc_dir
            ax.quiver(data['pos'][:,0], data['pos'][:,1], data['comp_asc_dir'][:,0], data['comp_asc_dir'][:,1], color="b", angles='xy', scale_units='xy', scale=1/ds)

    def OnClose(self, event):
        self.Destroy()

    def OnRedrawTimer(self, event):
        self.draw_ss()
        self.canvas.draw()


class SSPlotApp(wx.App):
    def __init__(self):
        super().__init__()

    def OnInit(self):
        self.main = SSFrame()

        self.main.Show()
        self.SetTopWindow(self.main)
        return True

def main():
    application = SSPlotApp()
    application.MainLoop()

if __name__ == '__main__':
    main()
