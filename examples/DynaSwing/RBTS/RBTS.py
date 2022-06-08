from pandas import read_json
from numpy import nan
import pandapower as pp
from math import sqrt, pi
try:
    import pplog as logging
except ImportError:
    import logging
logger = logging.getLogger(__name__)


def create_RBTS():
    """
    Create the Roy Billinton Test System from "A reliability test system for educational purposes-basic data" and
    Appendix A of "Reliability Assessment of Electric Power Systems Using Monte Carlo Methods"

    OUTPUT:
        **net** - The pandapower format network.
    """
    rbts = pp.create_empty_network()
    Ub = 230e3
    Sb = 100e6

    Ib = Sb/(sqrt(3)*Ub)
    Zb = Ub/(sqrt(3)*Ib)

    f = 50

    line_length_multi = 2
    capa_multi = 0.1 # To have more realistic ratios

    # Linedata
    # ohm_pu = (2* pi *50) * 1e-9 * Zb * nanofarad
    # nanofarad = ohm_pu / (2*pi*50) / 1e-9 / Zb
    # Line230kV
    line_data = {'c_nf_per_km': capa_multi * 0.0352/250 / (2*pi*f) / 1e-9 / Zb * 2, 'r_ohm_per_km': 0.1140/250 * Zb,
                 'x_ohm_per_km': 0.600/250 * Zb, 'max_i_ka': 0.71 * Ib}

    pp.create_std_type(rbts, line_data, 'Line230kV', element='line')

    # Different line rating for L1 and L6
    line_data = {'c_nf_per_km': capa_multi * 0.0352/250 / (2*pi*f) / 1e-9 / Zb * 2, 'r_ohm_per_km': 0.1140/250 * Zb,
                 'x_ohm_per_km': 0.600/250 * Zb, 'max_i_ka': 0.85 * Ib}

    pp.create_std_type(rbts, line_data, 'Line230kV_0.85', element='line')

    # Busses
    bus1 = pp.create_bus(rbts, name='Bus 1', vn_kv=230, type='b', zone='RBTS', max_vm_pu=1.03, min_vm_pu=0.9) # Vmin decreased from 0.97
    bus2 = pp.create_bus(rbts, name='Bus 2', vn_kv=230, type='b', zone='RBTS', max_vm_pu=1.05, min_vm_pu=0.5)
    bus3 = pp.create_bus(rbts, name='Bus 3', vn_kv=230, type='b', zone='RBTS', max_vm_pu=1.05, min_vm_pu=0.5)
    bus4 = pp.create_bus(rbts, name='Bus 4', vn_kv=230, type='b', zone='RBTS', max_vm_pu=1.05, min_vm_pu=0.5)
    bus5 = pp.create_bus(rbts, name='Bus 5', vn_kv=230, type='b', zone='RBTS', max_vm_pu=1.05, min_vm_pu=0.5)
    bus6 = pp.create_bus(rbts, name='Bus 6', vn_kv=230, type='b', zone='RBTS', max_vm_pu=1.05, min_vm_pu=0.5)

    # Lines
    pp.create_line(rbts, bus1, bus3, length_km=75 * line_length_multi,
                   std_type='Line230kV_0.85', max_loading_percent=100, name='Line 1-3a')
    pp.create_line(rbts, bus1, bus3, length_km=75 * line_length_multi,
                   std_type='Line230kV_0.85', max_loading_percent=100, name='Line 1-3b')
    pp.create_line(rbts, bus2, bus4, length_km=250 * line_length_multi,
                   std_type='Line230kV', max_loading_percent=100, name='Line 2-4a')
    pp.create_line(rbts, bus2, bus4, length_km=250 * line_length_multi,
                   std_type='Line230kV', max_loading_percent=100, name='Line 2-4b')
    pp.create_line(rbts, bus1, bus2, length_km=200 * line_length_multi,
                   std_type='Line230kV', max_loading_percent=100, name='Line 1-2')
    pp.create_line(rbts, bus3, bus4, length_km=50 * line_length_multi,
                   std_type='Line230kV', max_loading_percent=100, name='Line 3-4')
    pp.create_line(rbts, bus3, bus5, length_km=50 * line_length_multi,
                   std_type='Line230kV', max_loading_percent=100, name='Line 3-5')
    pp.create_line(rbts, bus4, bus5, length_km=50 * line_length_multi,
                   std_type='Line230kV', max_loading_percent=100, name='Line 4-5')
    pp.create_line(rbts, bus5, bus6, length_km=50 * line_length_multi,
                   std_type='Line230kV', max_loading_percent=100, name='Line 5-6')

    # Loads
    load = 145 # peak = 185
    pp.create_load(rbts, bus2, p_mw=0.1081*load, q_mvar=-0.1081*load * 0, name='Load 2') # Q = 20% of P -> power factor of 0.98
    pp.create_load(rbts, bus3, p_mw=0.4595*load, q_mvar=-0.4595*load * 0.15, name='Load 3')
    pp.create_load(rbts, bus4, p_mw=0.2162*load, q_mvar=-0.2162*load * 0.15, name='Load 4')
    pp.create_load(rbts, bus5, p_mw=0.1081*load, q_mvar=-0.1081*load * 0.15, name='Load 5')
    pp.create_load(rbts, bus6, p_mw=0.1081*load, q_mvar=-0.1081*load * 0.15, name='Load 6')

    # Generators, no minimum active power given
    # Assume 1$ = 1€
    pp.create_gen(rbts, bus1, p_mw=40, type='Thermal', min_p_mw=14, max_p_mw=40, min_q_mvar=-15, max_q_mvar=17, name='Generator 1')
    pp.create_poly_cost(rbts, 0, 'gen', cp1_eur_per_mw=12, cp0_eur=790e3/365/24)
    pp.create_gen(rbts, bus1, p_mw=40, type='Thermal', min_p_mw=14, max_p_mw=40, min_q_mvar=-15, max_q_mvar=17, name='Generator 2')
    pp.create_poly_cost(rbts, 1, 'gen', cp1_eur_per_mw=12, cp0_eur=790e3/365/24)
    pp.create_gen(rbts, bus1, p_mw=10, type='Thermal', min_p_mw=0, max_p_mw=10, min_q_mvar=0, max_q_mvar=7, name='Generator 3')
    pp.create_poly_cost(rbts, 2, 'gen', cp1_eur_per_mw=12.5, cp0_eur=600e3/365/24)
    pp.create_gen(rbts, bus1, p_mw=20, type='Thermal', min_p_mw=0, max_p_mw=20, min_q_mvar=-7, max_q_mvar=12, name='Generator 4')
    pp.create_poly_cost(rbts, 3, 'gen', cp1_eur_per_mw=12.25, cp0_eur=680e3/365/24)

    pp.create_gen(rbts, bus2, p_mw=5, type='Hydro', min_p_mw=0, max_p_mw=5, min_q_mvar=0, max_q_mvar=5, name='Generator 5')
    pp.create_poly_cost(rbts, 4, 'gen', cp1_eur_per_mw=0.5, cp0_eur=12.5e3/365/24)
    pp.create_gen(rbts, bus2, p_mw=5, type='Hydro', min_p_mw=0, max_p_mw=5, min_q_mvar=0, max_q_mvar=5, name='Generator 6')
    pp.create_poly_cost(rbts, 5, 'gen', cp1_eur_per_mw=0.5, cp0_eur=12.5e3/365/24)
    pp.create_gen(rbts, bus2, p_mw=40, type='Hydro', min_p_mw=0, max_p_mw=40, min_q_mvar=-15, max_q_mvar=17, name='Generator 7', slack=True)
    pp.create_poly_cost(rbts, 6, 'gen', cp1_eur_per_mw=0.5, cp0_eur=100e3/365/24)
    pp.create_gen(rbts, bus2, p_mw=20, type='Hydro', min_p_mw=0, max_p_mw=20, min_q_mvar=-7, max_q_mvar=12, name='Generator 8')
    pp.create_poly_cost(rbts, 7, 'gen', cp1_eur_per_mw=0.5, cp0_eur=50e3/365/24)
    pp.create_gen(rbts, bus2, p_mw=20, type='Hydro', min_p_mw=0, max_p_mw=20, min_q_mvar=-7, max_q_mvar=12, name='Generator 9')
    pp.create_poly_cost(rbts, 8, 'gen', cp1_eur_per_mw=0.5, cp0_eur=50e3/365/24)
    pp.create_gen(rbts, bus2, p_mw=20, type='Hydro', min_p_mw=0, max_p_mw=20, min_q_mvar=-7, max_q_mvar=12, name='Generator 10')
    pp.create_poly_cost(rbts, 9, 'gen', cp1_eur_per_mw=0.5, cp0_eur=50e3/365/24)
    pp.create_gen(rbts, bus2, p_mw=20, type='Hydro', min_p_mw=0, max_p_mw=20, min_q_mvar=-7, max_q_mvar=12, name='Generator 11')
    pp.create_poly_cost(rbts, 10, 'gen', cp1_eur_per_mw=0.5, cp0_eur=50e3/365/24)

    pp.runopp(rbts, verbose=True)

    # It does not seem possible to allow generators to be turned off in the OPF formulation
    indexes = []
    for i in range(len(rbts.res_gen.p_mw)):
        if rbts.res_gen.p_mw[i] < 1: # Generators that produce less than 1 MW are shut off
            indexes.append(i)
    for i in indexes:
        rbts.gen.in_service[i] = False

    pp.runopp(rbts, verbose=True) # Rerun OPF without the non-necessary generators (assume there is still a solution)

    # Set the generator setpoints to follow the OPF results (the converter does not read the OPF results)
    rbts.gen.vm_pu = rbts.res_gen.vm_pu
    rbts.gen.p_mw = rbts.res_gen.p_mw

    pp.converter.to_mpc(rbts, "RBTS_ACOPF_145_L2_shunt_typical.mat")

    print("Cost: %f" % rbts.res_cost)

    # Run
    # itools loadflow --case-file RBTS_ACOPF_peak.mat --output-case-file RBTS_ACOPF_peak.xiidm --output-case-format XIIDM
    # to convert to iidm

    # But should first add
    # import-export-parameters-default-value:
    #   matpower.import.ignore-base-voltage: false
    # to the config.yml, otherwise powsybl sets all base voltages to 1 (and thus transform the impedance to "secondary" values)

    # Problem: matpower does not have the names of the generators => automatic name in iidm
    # => Name changes depending on which generators are on => dyd file has to be changed
    # Issue will probably solved when going to PowerModels.jl

    return rbts

if __name__ == "__main__":
    create_RBTS()
