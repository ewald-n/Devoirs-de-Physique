classdef AccParams
    properties
        option
        m_balle
        g
        k_visc
        S_magnus
        w_init
    end

    methods
        function obj = AccParams(option, m_balle, g, k_visc, S_magnus, w_init)
            obj.option = option;
            obj.m_balle = m_balle;
            obj.g = g;
            obj.k_visc = k_visc;
            obj.S_magnus = S_magnus;
            obj.w_init = w_init;
        end
    end
end
