// This is a main entry point for configuring a wire-cell CLI job to
// simulate SBND.  It is simplest signal-only simulation with
// one set of nominal field response function.  
local g = import 'pgraph.jsonnet';
local f = import 'pgrapher/common/funcs.jsonnet';
local wc = import 'wirecell.jsonnet';
local io = import 'pgrapher/common/fileio.jsonnet';
local tools_maker = import 'pgrapher/common/tools.jsonnet';
local base = import 'pgrapher/experiment/sbnd/simparams.jsonnet';
local params = base {
  lar: super.lar {
    // Longitudinal diffusion constant
    DL: std.extVar('DL') * wc.cm2 / wc.s,
    // Transverse diffusion constant
    DT: std.extVar('DT') * wc.cm2 / wc.s,
    // Electron lifetime
    lifetime: std.extVar('lifetime') * wc.ms,
    // Electron drift speed, assumes a certain applied E-field
    drift_speed: std.extVar('driftSpeed') * wc.mm / wc.us,
  },
};
local tools = tools_maker(params);
local sim_maker = import 'pgrapher/experiment/sbnd/sim.jsonnet';
local sim = sim_maker(params, tools);
local nanodes = std.length(tools.anodes);
local anode_iota = std.range(0, nanodes - 1);
local output = 'wct-sim-ideal-sig.npz';
//local depos = g.join_sources(g.pnode({type:"DepoMerger", name:"BlipTrackJoiner"}, nin=2, nout=1),
//                             [sim.ar39(), sim.tracks(tracklist)]);
// local depos = sim.tracks(tracklist, step=1.0 * wc.mm);
local wcls_maker = import "pgrapher/ui/wcls/nodes.jsonnet";
local wcls = wcls_maker(params, tools);
// added Ewerton 2023-03-14
local wcls_input = {
    depos: wcls.input.depos(name="", art_tag=std.extVar('inputTag')), //commented Ewerton 2023-03-15
    deposet: g.pnode({
            type: 'wclsSimDepoSetSource',
            name: "",
            data: {
                model: "",
                scale: -1, //scale is -1 to correct a sign error in the SimDepoSource converter.
                art_tag: std.extVar('inputTag'), //name of upstream art producer of depos "label:instance:processName"
                id_is_track: false,
                assn_art_tag: "",
            },
        }, nin=0, nout=1),
};
// Collect all the wc/ls output converters for use below.  Note the
// "name" MUST match what is used in theh "outputers" parameter in the
// FHiCL that loads this file.
local mega_anode = {
  type: 'MegaAnodePlane',
  name: 'meganodes',
  data: {
    anodes_tn: [wc.tn(anode) for anode in tools.anodes],
  },
};
local wcls_output = {
  // ADC output from simulation
  // sim_digits: wcls.output.digits(name="simdigits", tags=["orig"]),
  sim_digits: g.pnode({
    type: 'wclsFrameSaver',
    name: 'simdigits',
    data: {
      // anode: wc.tn(tools.anode),
      anode: wc.tn(mega_anode),
      digitize: true,  // true means save as RawDigit, else recob::Wire
      frame_tags: ['daq'],
      // nticks: params.daq.nticks,
      // chanmaskmaps: ['bad'],
      pedestal_mean: 'native',
    },
  }, nin=1, nout=1, uses=[mega_anode]),
};
//local deposio = io.numpy.depos(output);
local drifter = sim.drifter;
// added Ewerton 2023-03-14
local setdrifter = g.pnode({
            type: 'DepoSetDrifter',
            data: {
                drifter: "Drifter"
            }
        }, nin=1, nout=1,
        uses=[drifter]);
local bagger = sim.make_bagger();
// signal plus noise pipelines
// local sn_pipes = sim.signal_pipelines;
local sn_pipes = sim.splusn_pipelines;
local perfect = import 'pgrapher/experiment/sbnd/chndb-perfect.jsonnet';
local chndb = [{
  type: 'OmniChannelNoiseDB',
  name: 'ocndbperfect%d' % n,
  data: perfect(params, tools.anodes[n], tools.field, n){dft:wc.tn(tools.dft)},
  uses: [tools.anodes[n], tools.field, tools.dft],
} for n in anode_iota];

local rng = tools.random;
local wcls_depoflux_writer = g.pnode({
  type: 'wclsDepoFluxWriter',
  name: 'postdrift',
  data: {
    anodes: [wc.tn(anode) for anode in tools.anodes],
    field_response: wc.tn(tools.field),
    tick: 0.5 * wc.us,
    window_start: 0.0 * wc.ms,
    window_duration: self.tick * 3400,
    nsigma: 3.0,

    reference_time: -1700 * wc.us,

    energy: 1, # equivalent to use_energy = true
    simchan_label: 'simpleSC',
    sed_label: 'ionandscint',
    sparse: false,
  },
}, nin=1, nout=1, uses=tools.anodes + [tools.field]);
// local magoutput = 'sbnd-data-check.root';
// local magnify = import 'pgrapher/experiment/sbnd/magnify-sinks.jsonnet';
// local sinks = magnify(tools, magoutput);
local multipass = [
  g.pipeline([
               sn_pipes[n],
               // sinks.orig_pipe[n],

             ],
             'multipass%d' % n)
  for n in anode_iota
];
local outtags = ['orig%d' % n for n in anode_iota];
local bi_manifold = f.fanpipe('DepoSetFanout', multipass, 'FrameFanin', 'sn_mag_nf', outtags);
// local bi_manifold = f.fanpipe('DepoFanout', multipass, 'FrameFanin', 'sn_mag_nf', outtags);
local retagger = g.pnode({
  type: 'Retagger',
  data: {
    // Note: retagger keeps tag_rules an array to be like frame fanin/fanout.
    tag_rules: [{
      // Retagger also handles "frame" and "trace" like fanin/fanout
      // merge separately all traces like gaussN to gauss.
      frame: {
        '.*': 'orig',
      },
      merge: {
        'orig\\d': 'daq',
      },
    }],
  },
}, nin=1, nout=1);
//local frameio = io.numpy.frames(output);
local sink = sim.frame_sink;
local graph = g.pipeline([wcls_input.deposet, setdrifter, wcls_depoflux_writer, bi_manifold, retagger, wcls_output.sim_digits, sink]); //<- SimDepoFluxWriter and DepoSetDrifter
local app = {
  type: 'TbbFlow', //TbbFlow Pgrapher changed Ewerton 2023-03-14
  data: {
    edges: g.edges(graph),
  },
};
// Finally, the configuration sequence which is emitted.
g.uses(graph) + [app]

