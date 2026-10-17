/* Generato da genera_vettori.py — non modificare a mano.
 * La configurazione dell'ambiente, senza i vettori di prova. */
#ifndef CONFIG_BORDO_H
#define CONFIG_BORDO_H

#include "osservazione.h"

static inline osservazione_config_t config_bordo(void)
{
    osservazione_config_t c = {
        .step_dt = 0.0399999991f,
        .integral_alpha = 0.986755133f,
        .integral_clamp = 0.5f,
        .integral_gate_dist = 1.0f,
        .integral_obs_scale = 0.5f,
        .integral_keep_on_advance = 0.5f,
        .tof_obs_scale = 8.80000019f,
        .tof_max_range = 8.80000019f,
        .map_logodds_clamp = 4.0f,
        /* IL PRIOR DIPENDE DAL BRACCIO (3/10).
         *
         *   uni_arco_v6   map_prior_ignoto = 0.10
         *   uni_vae_v3    map_prior_ignoto = 0.15
         *
         * E' la "prudenza verso l'ignoto": quanto vale una cella mai vista, o
         * dimenticata dal decadimento. Fino al 3/10 qui c'era 0.10 fisso,
         * quindi TUTTE le corse HIL del ramo VAE hanno usato il prior del v6
         * su 1024 dei 1148 numeri dell'osservazione. Nessun errore, nessun
         * avviso: solo una mappa piu' ottimista di quella con cui la policy
         * e' stata addestrata.
         *
         * Il simbolo c'e' gia': lo stesso che sceglie il ramo in
         * ciclo_seriale.c, quindi la scelta segue il progetto compilato. */
#if defined(CICLO_SERIALE_VAE)
        .map_prior = 0.150000006f,
#else
        .map_prior = 0.100000001f,
#endif
        .map_decay = 0.990049839f,
        .preview_attenuazione_dist = 0.0f,
        .n_waypoints = 4,
        .wp_preview_horizon = 4,
        .tof_sectori_v = 3,
        .tof_settori_h = 4,
    };
    return c;
}

#endif
