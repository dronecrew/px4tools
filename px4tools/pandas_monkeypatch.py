import pandas

if pandas.__version__ < "0.19.99":

    try:

        print("attempting to monkey patch pandas timedelta series plotting")

        from matplotlib.ticker import Formatter
        import pandas.tools
        import pandas.tseries.plotting
        import pylab
        import numpy as np
        from pandas.tseries.plotting import TimeSeries_DateLocator, TimeSeries_DateFormatter

        class TimeSeries_TimedeltaFormatter(Formatter):
            """
            Formats the ticks along an axis controlled by a :class:`TimedeltaIndex`.
            """

            @staticmethod
            def format_timedelta_ticks(x, pos, n_decimals):
                """
                Convert seconds to 'D days HH:MM:SS.F'
                """
                s, ns = divmod(x, 1e9)
                m, s = divmod(s, 60)
                h, m = divmod(m, 60)
                d, h = divmod(h, 24)
                decimals = int(ns * 10**(n_decimals - 9))
                s = r'{:02d}:{:02d}:{:02d}'.format(int(h), int(m), int(s))
                if n_decimals > 0:
                    s += '.{{:0{:0d}d}}'.format(n_decimals).format(decimals)
                if d != 0:
                    s = '{:d} days '.format(int(d)) + s
                return s

            def __call__(self, x, pos=0):
                (vmin, vmax) = tuple(self.axis.get_view_interval())
                n_decimals = int(np.ceil(np.log10(100 * 1e9 / (vmax - vmin))))
                if n_decimals > 9:
                    n_decimals = 9
                return self.format_timedelta_ticks(x, pos, n_decimals)

        def _ts_plot(cls, ax, x, data, style=None, **kwds):
            from pandas.tseries.plotting import (_maybe_resample,
                                                 _decorate_axes,
                                                 format_dateaxis)
            # accept x to be consistent with normal plot func,
            # x is not passed to tsplot as it uses data.index as x coordinate
            # column_num must be in kwds for stacking purpose
            freq, data = _maybe_resample(data, ax, kwds)

            # Set ax with freq info
            _decorate_axes(ax, freq, kwds)
            # digging deeper
            if hasattr(ax, 'left_ax'):
                _decorate_axes(ax.left_ax, freq, kwds)
            if hasattr(ax, 'right_ax'):
                _decorate_axes(ax.right_ax, freq, kwds)
            ax._plot_data.append((data, cls._kind, kwds))

            lines = cls._plot(ax, data.index, data.values, style=style, **kwds)
            # set date formatter, locators and rescale limits
            format_dateaxis(ax, ax.freq, data.index)
            return lines

        def format_dateaxis(subplot, freq, index):
            """
            Pretty-formats the date axis (x-axis).

            Major and minor ticks are automatically set for the frequency of the
            current underlying series.  As the dynamic mode is activated by
            default, changing the limits of the x axis will intelligently change
            the positions of the ticks.
            """

            from pandas import PeriodIndex, TimedeltaIndex

            # handle index specific formatting
            # Note: DatetimeIndex does not use this
            # interface. DatetimeIndex uses matplotlib.date directly
            if isinstance(index, PeriodIndex):

                majlocator = TimeSeries_DateLocator(freq, dynamic_mode=True,
                                                    minor_locator=False,
                                                    plot_obj=subplot)
                minlocator = TimeSeries_DateLocator(freq, dynamic_mode=True,
                                                    minor_locator=True,
                                                    plot_obj=subplot)
                subplot.xaxis.set_major_locator(majlocator)
                subplot.xaxis.set_minor_locator(minlocator)

                majformatter = TimeSeries_DateFormatter(freq, dynamic_mode=True,
                                                        minor_locator=False,
                                                        plot_obj=subplot)
                minformatter = TimeSeries_DateFormatter(freq, dynamic_mode=True,
                                                        minor_locator=True,
                                                        plot_obj=subplot)
                subplot.xaxis.set_major_formatter(majformatter)
                subplot.xaxis.set_minor_formatter(minformatter)

                # x and y coord info
                subplot.format_coord = lambda t, y: (
                    "t = {0}  y = {1:8f}".format(Period(ordinal=int(t), freq=freq), y))

            elif isinstance(index, TimedeltaIndex):
                subplot.xaxis.set_major_formatter(
                    TimeSeries_TimedeltaFormatter())
            else:
                raise TypeError('index type not supported')

            pylab.draw_if_interactive()

        pandas.tseries.plotting.format_dateaxis = format_dateaxis
        pandas.tools.plotting.LinePlot._ts_plot = _ts_plot

        print("monkey patch succeeded")

    except Exception as e:
        print("monkey patch failed", e)

#  vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : 
