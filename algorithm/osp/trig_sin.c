/*
 * (C) Copyright 2015 HY Research LLC
 *     Author: hy-git@hy-research.com
 *
 * Apache License.
 *
 * Trig look up table.
 */



static struct TRIG_SIN {
    Q15_t sin;
    Q15_t ang;
} trig_sin[] = {
{FP_to_Q15(0.00000), FP_to_Q15(0.00000)},
{FP_to_Q15(0.00500), FP_to_Q15(0.00500)},
{FP_to_Q15(0.01000), FP_to_Q15(0.01000)},
{FP_to_Q15(0.01500), FP_to_Q15(0.01500)},
{FP_to_Q15(0.02000), FP_to_Q15(0.02000)},
{FP_to_Q15(0.02500), FP_to_Q15(0.02500)},
{FP_to_Q15(0.03000), FP_to_Q15(0.03000)},
{FP_to_Q15(0.03499), FP_to_Q15(0.03500)},
{FP_to_Q15(0.03999), FP_to_Q15(0.04000)},
{FP_to_Q15(0.04498), FP_to_Q15(0.04500)},
{FP_to_Q15(0.04998), FP_to_Q15(0.05000)},
{FP_to_Q15(0.05497), FP_to_Q15(0.05500)},
{FP_to_Q15(0.05996), FP_to_Q15(0.06000)},
{FP_to_Q15(0.06495), FP_to_Q15(0.06500)},
{FP_to_Q15(0.06994), FP_to_Q15(0.07000)},
{FP_to_Q15(0.07493), FP_to_Q15(0.07500)},
{FP_to_Q15(0.07991), FP_to_Q15(0.08000)},
{FP_to_Q15(0.08490), FP_to_Q15(0.08500)},
{FP_to_Q15(0.08988), FP_to_Q15(0.09000)},
{FP_to_Q15(0.09486), FP_to_Q15(0.09500)},
{FP_to_Q15(0.09983), FP_to_Q15(0.10000)},
{FP_to_Q15(0.10481), FP_to_Q15(0.10500)},
{FP_to_Q15(0.10978), FP_to_Q15(0.11000)},
{FP_to_Q15(0.11475), FP_to_Q15(0.11500)},
{FP_to_Q15(0.11971), FP_to_Q15(0.12000)},
{FP_to_Q15(0.12467), FP_to_Q15(0.12500)},
{FP_to_Q15(0.12963), FP_to_Q15(0.13000)},
{FP_to_Q15(0.13459), FP_to_Q15(0.13500)},
{FP_to_Q15(0.13954), FP_to_Q15(0.14000)},
{FP_to_Q15(0.14449), FP_to_Q15(0.14500)},
{FP_to_Q15(0.14944), FP_to_Q15(0.15000)},
{FP_to_Q15(0.15438), FP_to_Q15(0.15500)},
{FP_to_Q15(0.15932), FP_to_Q15(0.16000)},
{FP_to_Q15(0.16425), FP_to_Q15(0.16500)},
{FP_to_Q15(0.16918), FP_to_Q15(0.17000)},
{FP_to_Q15(0.17411), FP_to_Q15(0.17500)},
{FP_to_Q15(0.17903), FP_to_Q15(0.18000)},
{FP_to_Q15(0.18395), FP_to_Q15(0.18500)},
{FP_to_Q15(0.18886), FP_to_Q15(0.19000)},
{FP_to_Q15(0.19377), FP_to_Q15(0.19500)},
{FP_to_Q15(0.19867), FP_to_Q15(0.20000)},
{FP_to_Q15(0.20357), FP_to_Q15(0.20500)},
{FP_to_Q15(0.20846), FP_to_Q15(0.21000)},
{FP_to_Q15(0.21335), FP_to_Q15(0.21500)},
{FP_to_Q15(0.21823), FP_to_Q15(0.22000)},
{FP_to_Q15(0.22311), FP_to_Q15(0.22500)},
{FP_to_Q15(0.22798), FP_to_Q15(0.23000)},
{FP_to_Q15(0.23284), FP_to_Q15(0.23500)},
{FP_to_Q15(0.23770), FP_to_Q15(0.24000)},
{FP_to_Q15(0.24256), FP_to_Q15(0.24500)},
{FP_to_Q15(0.24740), FP_to_Q15(0.25000)},
{FP_to_Q15(0.25225), FP_to_Q15(0.25500)},
{FP_to_Q15(0.25708), FP_to_Q15(0.26000)},
{FP_to_Q15(0.26191), FP_to_Q15(0.26500)},
{FP_to_Q15(0.26673), FP_to_Q15(0.27000)},
{FP_to_Q15(0.27155), FP_to_Q15(0.27500)},
{FP_to_Q15(0.27636), FP_to_Q15(0.28000)},
{FP_to_Q15(0.28116), FP_to_Q15(0.28500)},
{FP_to_Q15(0.28595), FP_to_Q15(0.29000)},
{FP_to_Q15(0.29074), FP_to_Q15(0.29500)},
{FP_to_Q15(0.29552), FP_to_Q15(0.30000)},
{FP_to_Q15(0.30029), FP_to_Q15(0.30500)},
{FP_to_Q15(0.30506), FP_to_Q15(0.31000)},
{FP_to_Q15(0.30982), FP_to_Q15(0.31500)},
{FP_to_Q15(0.31457), FP_to_Q15(0.32000)},
{FP_to_Q15(0.31931), FP_to_Q15(0.32500)},
{FP_to_Q15(0.32404), FP_to_Q15(0.33000)},
{FP_to_Q15(0.32877), FP_to_Q15(0.33500)},
{FP_to_Q15(0.33349), FP_to_Q15(0.34000)},
{FP_to_Q15(0.33820), FP_to_Q15(0.34500)},
{FP_to_Q15(0.34290), FP_to_Q15(0.35000)},
{FP_to_Q15(0.34759), FP_to_Q15(0.35500)},
{FP_to_Q15(0.35227), FP_to_Q15(0.36000)},
{FP_to_Q15(0.35695), FP_to_Q15(0.36500)},
{FP_to_Q15(0.36162), FP_to_Q15(0.37000)},
{FP_to_Q15(0.36627), FP_to_Q15(0.37500)},
{FP_to_Q15(0.37092), FP_to_Q15(0.38000)},
{FP_to_Q15(0.37556), FP_to_Q15(0.38500)},
{FP_to_Q15(0.38019), FP_to_Q15(0.39000)},
{FP_to_Q15(0.38481), FP_to_Q15(0.39500)},
{FP_to_Q15(0.38942), FP_to_Q15(0.40000)},
{FP_to_Q15(0.39402), FP_to_Q15(0.40500)},
{FP_to_Q15(0.39861), FP_to_Q15(0.41000)},
{FP_to_Q15(0.40319), FP_to_Q15(0.41500)},
{FP_to_Q15(0.40776), FP_to_Q15(0.42000)},
{FP_to_Q15(0.41232), FP_to_Q15(0.42500)},
{FP_to_Q15(0.41687), FP_to_Q15(0.43000)},
{FP_to_Q15(0.42141), FP_to_Q15(0.43500)},
{FP_to_Q15(0.42594), FP_to_Q15(0.44000)},
{FP_to_Q15(0.43046), FP_to_Q15(0.44500)},
{FP_to_Q15(0.43497), FP_to_Q15(0.45000)},
{FP_to_Q15(0.43946), FP_to_Q15(0.45500)},
{FP_to_Q15(0.44395), FP_to_Q15(0.46000)},
{FP_to_Q15(0.44842), FP_to_Q15(0.46500)},
{FP_to_Q15(0.45289), FP_to_Q15(0.47000)},
{FP_to_Q15(0.45734), FP_to_Q15(0.47500)},
{FP_to_Q15(0.46178), FP_to_Q15(0.48000)},
{FP_to_Q15(0.46621), FP_to_Q15(0.48500)},
{FP_to_Q15(0.47063), FP_to_Q15(0.49000)},
{FP_to_Q15(0.47503), FP_to_Q15(0.49500)},
{FP_to_Q15(0.47943), FP_to_Q15(0.50000)},
{FP_to_Q15(0.48381), FP_to_Q15(0.50500)},
{FP_to_Q15(0.48818), FP_to_Q15(0.51000)},
{FP_to_Q15(0.49253), FP_to_Q15(0.51500)},
{FP_to_Q15(0.49688), FP_to_Q15(0.52000)},
{FP_to_Q15(0.50121), FP_to_Q15(0.52500)},
{FP_to_Q15(0.50553), FP_to_Q15(0.53000)},
{FP_to_Q15(0.50984), FP_to_Q15(0.53500)},
{FP_to_Q15(0.51414), FP_to_Q15(0.54000)},
{FP_to_Q15(0.51842), FP_to_Q15(0.54500)},
{FP_to_Q15(0.52269), FP_to_Q15(0.55000)},
{FP_to_Q15(0.52694), FP_to_Q15(0.55500)},
{FP_to_Q15(0.53119), FP_to_Q15(0.56000)},
{FP_to_Q15(0.53542), FP_to_Q15(0.56500)},
{FP_to_Q15(0.53963), FP_to_Q15(0.57000)},
{FP_to_Q15(0.54383), FP_to_Q15(0.57500)},
{FP_to_Q15(0.54802), FP_to_Q15(0.58000)},
{FP_to_Q15(0.55220), FP_to_Q15(0.58500)},
{FP_to_Q15(0.55636), FP_to_Q15(0.59000)},
{FP_to_Q15(0.56051), FP_to_Q15(0.59500)},
{FP_to_Q15(0.56464), FP_to_Q15(0.60000)},
{FP_to_Q15(0.56876), FP_to_Q15(0.60500)},
{FP_to_Q15(0.57287), FP_to_Q15(0.61000)},
{FP_to_Q15(0.57696), FP_to_Q15(0.61500)},
{FP_to_Q15(0.58104), FP_to_Q15(0.62000)},
{FP_to_Q15(0.58510), FP_to_Q15(0.62500)},
{FP_to_Q15(0.58914), FP_to_Q15(0.63000)},
{FP_to_Q15(0.59318), FP_to_Q15(0.63500)},
{FP_to_Q15(0.59720), FP_to_Q15(0.64000)},
{FP_to_Q15(0.60120), FP_to_Q15(0.64500)},
{FP_to_Q15(0.60519), FP_to_Q15(0.65000)},
{FP_to_Q15(0.60916), FP_to_Q15(0.65500)},
{FP_to_Q15(0.61312), FP_to_Q15(0.66000)},
{FP_to_Q15(0.61706), FP_to_Q15(0.66500)},
{FP_to_Q15(0.62099), FP_to_Q15(0.67000)},
{FP_to_Q15(0.62490), FP_to_Q15(0.67500)},
{FP_to_Q15(0.62879), FP_to_Q15(0.68000)},
{FP_to_Q15(0.63267), FP_to_Q15(0.68500)},
{FP_to_Q15(0.63654), FP_to_Q15(0.69000)},
{FP_to_Q15(0.64039), FP_to_Q15(0.69500)},
{FP_to_Q15(0.64422), FP_to_Q15(0.70000)},
{FP_to_Q15(0.64803), FP_to_Q15(0.70500)},
{FP_to_Q15(0.65183), FP_to_Q15(0.71000)},
{FP_to_Q15(0.65562), FP_to_Q15(0.71500)},
{FP_to_Q15(0.65938), FP_to_Q15(0.72000)},
{FP_to_Q15(0.66314), FP_to_Q15(0.72500)},
{FP_to_Q15(0.66687), FP_to_Q15(0.73000)},
{FP_to_Q15(0.67059), FP_to_Q15(0.73500)},
{FP_to_Q15(0.67429), FP_to_Q15(0.74000)},
{FP_to_Q15(0.67797), FP_to_Q15(0.74500)},
{FP_to_Q15(0.68164), FP_to_Q15(0.75000)},
{FP_to_Q15(0.68529), FP_to_Q15(0.75500)},
{FP_to_Q15(0.68892), FP_to_Q15(0.76000)},
{FP_to_Q15(0.69254), FP_to_Q15(0.76500)},
{FP_to_Q15(0.69614), FP_to_Q15(0.77000)},
{FP_to_Q15(0.69972), FP_to_Q15(0.77500)},
{FP_to_Q15(0.70328), FP_to_Q15(0.78000)},
{FP_to_Q15(0.70683), FP_to_Q15(0.78500)},
{FP_to_Q15(0.71035), FP_to_Q15(0.79000)},
{FP_to_Q15(0.71386), FP_to_Q15(0.79500)},
{FP_to_Q15(0.71736), FP_to_Q15(0.80000)},
{FP_to_Q15(0.72083), FP_to_Q15(0.80500)},
{FP_to_Q15(0.72429), FP_to_Q15(0.81000)},
{FP_to_Q15(0.72773), FP_to_Q15(0.81500)},
{FP_to_Q15(0.73115), FP_to_Q15(0.82000)},
{FP_to_Q15(0.73455), FP_to_Q15(0.82500)},
{FP_to_Q15(0.73793), FP_to_Q15(0.83000)},
{FP_to_Q15(0.74130), FP_to_Q15(0.83500)},
{FP_to_Q15(0.74464), FP_to_Q15(0.84000)},
{FP_to_Q15(0.74797), FP_to_Q15(0.84500)},
{FP_to_Q15(0.75128), FP_to_Q15(0.85000)},
{FP_to_Q15(0.75457), FP_to_Q15(0.85500)},
{FP_to_Q15(0.75784), FP_to_Q15(0.86000)},
{FP_to_Q15(0.76110), FP_to_Q15(0.86500)},
{FP_to_Q15(0.76433), FP_to_Q15(0.87000)},
{FP_to_Q15(0.76754), FP_to_Q15(0.87500)},
{FP_to_Q15(0.77074), FP_to_Q15(0.88000)},
{FP_to_Q15(0.77391), FP_to_Q15(0.88500)},
{FP_to_Q15(0.77707), FP_to_Q15(0.89000)},
{FP_to_Q15(0.78021), FP_to_Q15(0.89500)},
{FP_to_Q15(0.78333), FP_to_Q15(0.90000)},
{FP_to_Q15(0.78643), FP_to_Q15(0.90500)},
{FP_to_Q15(0.78950), FP_to_Q15(0.91000)},
{FP_to_Q15(0.79256), FP_to_Q15(0.91500)},
{FP_to_Q15(0.79560), FP_to_Q15(0.92000)},
{FP_to_Q15(0.79862), FP_to_Q15(0.92500)},
{FP_to_Q15(0.80162), FP_to_Q15(0.93000)},
{FP_to_Q15(0.80460), FP_to_Q15(0.93500)},
{FP_to_Q15(0.80756), FP_to_Q15(0.94000)},
{FP_to_Q15(0.81050), FP_to_Q15(0.94500)},
{FP_to_Q15(0.81342), FP_to_Q15(0.95000)},
{FP_to_Q15(0.81631), FP_to_Q15(0.95500)},
{FP_to_Q15(0.81919), FP_to_Q15(0.96000)},
{FP_to_Q15(0.82205), FP_to_Q15(0.96500)},
{FP_to_Q15(0.82489), FP_to_Q15(0.97000)},
{FP_to_Q15(0.82770), FP_to_Q15(0.97500)},
{FP_to_Q15(0.83050), FP_to_Q15(0.98000)},
{FP_to_Q15(0.83327), FP_to_Q15(0.98500)},
{FP_to_Q15(0.83603), FP_to_Q15(0.99000)},
{FP_to_Q15(0.83876), FP_to_Q15(0.99500)},
{FP_to_Q15(0.84147), FP_to_Q15(1.00000)},
{FP_to_Q15(0.84416), FP_to_Q15(1.00500)},
{FP_to_Q15(0.84683), FP_to_Q15(1.01000)},
{FP_to_Q15(0.84948), FP_to_Q15(1.01500)},
{FP_to_Q15(0.85211), FP_to_Q15(1.02000)},
{FP_to_Q15(0.85471), FP_to_Q15(1.02500)},
{FP_to_Q15(0.85730), FP_to_Q15(1.03000)},
{FP_to_Q15(0.85986), FP_to_Q15(1.03500)},
{FP_to_Q15(0.86240), FP_to_Q15(1.04000)},
{FP_to_Q15(0.86492), FP_to_Q15(1.04500)},
{FP_to_Q15(0.86742), FP_to_Q15(1.05000)},
{FP_to_Q15(0.86990), FP_to_Q15(1.05500)},
{FP_to_Q15(0.87236), FP_to_Q15(1.06000)},
{FP_to_Q15(0.87479), FP_to_Q15(1.06500)},
{FP_to_Q15(0.87720), FP_to_Q15(1.07000)},
{FP_to_Q15(0.87959), FP_to_Q15(1.07500)},
{FP_to_Q15(0.88196), FP_to_Q15(1.08000)},
{FP_to_Q15(0.88430), FP_to_Q15(1.08500)},
{FP_to_Q15(0.88663), FP_to_Q15(1.09000)},
{FP_to_Q15(0.88893), FP_to_Q15(1.09500)},
{FP_to_Q15(0.89121), FP_to_Q15(1.10000)},
{FP_to_Q15(0.89346), FP_to_Q15(1.10500)},
{FP_to_Q15(0.89570), FP_to_Q15(1.11000)},
{FP_to_Q15(0.89791), FP_to_Q15(1.11500)},
{FP_to_Q15(0.90010), FP_to_Q15(1.12000)},
{FP_to_Q15(0.90227), FP_to_Q15(1.12500)},
{FP_to_Q15(0.90441), FP_to_Q15(1.13000)},
{FP_to_Q15(0.90653), FP_to_Q15(1.13500)},
{FP_to_Q15(0.90863), FP_to_Q15(1.14000)},
{FP_to_Q15(0.91071), FP_to_Q15(1.14500)},
{FP_to_Q15(0.91276), FP_to_Q15(1.15000)},
{FP_to_Q15(0.91479), FP_to_Q15(1.15500)},
{FP_to_Q15(0.91680), FP_to_Q15(1.16000)},
{FP_to_Q15(0.91879), FP_to_Q15(1.16500)},
{FP_to_Q15(0.92075), FP_to_Q15(1.17000)},
{FP_to_Q15(0.92269), FP_to_Q15(1.17500)},
{FP_to_Q15(0.92461), FP_to_Q15(1.18000)},
{FP_to_Q15(0.92650), FP_to_Q15(1.18500)},
{FP_to_Q15(0.92837), FP_to_Q15(1.19000)},
{FP_to_Q15(0.93022), FP_to_Q15(1.19500)},
{FP_to_Q15(0.93204), FP_to_Q15(1.20000)},
{FP_to_Q15(0.93384), FP_to_Q15(1.20500)},
{FP_to_Q15(0.93562), FP_to_Q15(1.21000)},
{FP_to_Q15(0.93737), FP_to_Q15(1.21500)},
{FP_to_Q15(0.93910), FP_to_Q15(1.22000)},
{FP_to_Q15(0.94081), FP_to_Q15(1.22500)},
{FP_to_Q15(0.94249), FP_to_Q15(1.23000)},
{FP_to_Q15(0.94415), FP_to_Q15(1.23500)},
{FP_to_Q15(0.94578), FP_to_Q15(1.24000)},
{FP_to_Q15(0.94740), FP_to_Q15(1.24500)},
{FP_to_Q15(0.94898), FP_to_Q15(1.25000)},
{FP_to_Q15(0.95055), FP_to_Q15(1.25500)},
{FP_to_Q15(0.95209), FP_to_Q15(1.26000)},
{FP_to_Q15(0.95361), FP_to_Q15(1.26500)},
{FP_to_Q15(0.95510), FP_to_Q15(1.27000)},
{FP_to_Q15(0.95657), FP_to_Q15(1.27500)},
{FP_to_Q15(0.95802), FP_to_Q15(1.28000)},
{FP_to_Q15(0.95944), FP_to_Q15(1.28500)},
{FP_to_Q15(0.96084), FP_to_Q15(1.29000)},
{FP_to_Q15(0.96221), FP_to_Q15(1.29500)},
{FP_to_Q15(0.96356), FP_to_Q15(1.30000)},
{FP_to_Q15(0.96488), FP_to_Q15(1.30500)},
{FP_to_Q15(0.96618), FP_to_Q15(1.31000)},
{FP_to_Q15(0.96746), FP_to_Q15(1.31500)},
{FP_to_Q15(0.96872), FP_to_Q15(1.32000)},
{FP_to_Q15(0.96994), FP_to_Q15(1.32500)},
{FP_to_Q15(0.97115), FP_to_Q15(1.33000)},
{FP_to_Q15(0.97233), FP_to_Q15(1.33500)},
{FP_to_Q15(0.97348), FP_to_Q15(1.34000)},
{FP_to_Q15(0.97462), FP_to_Q15(1.34500)},
{FP_to_Q15(0.97572), FP_to_Q15(1.35000)},
{FP_to_Q15(0.97681), FP_to_Q15(1.35500)},
{FP_to_Q15(0.97786), FP_to_Q15(1.36000)},
{FP_to_Q15(0.97890), FP_to_Q15(1.36500)},
{FP_to_Q15(0.97991), FP_to_Q15(1.37000)},
{FP_to_Q15(0.98089), FP_to_Q15(1.37500)},
{FP_to_Q15(0.98185), FP_to_Q15(1.38000)},
{FP_to_Q15(0.98279), FP_to_Q15(1.38500)},
{FP_to_Q15(0.98370), FP_to_Q15(1.39000)},
{FP_to_Q15(0.98459), FP_to_Q15(1.39500)},
{FP_to_Q15(0.98545), FP_to_Q15(1.40000)},
{FP_to_Q15(0.98629), FP_to_Q15(1.40500)},
{FP_to_Q15(0.98710), FP_to_Q15(1.41000)},
{FP_to_Q15(0.98789), FP_to_Q15(1.41500)},
{FP_to_Q15(0.98865), FP_to_Q15(1.42000)},
{FP_to_Q15(0.98939), FP_to_Q15(1.42500)},
{FP_to_Q15(0.99010), FP_to_Q15(1.43000)},
{FP_to_Q15(0.99079), FP_to_Q15(1.43500)},
{FP_to_Q15(0.99146), FP_to_Q15(1.44000)},
{FP_to_Q15(0.99210), FP_to_Q15(1.44500)},
{FP_to_Q15(0.99271), FP_to_Q15(1.45000)},
{FP_to_Q15(0.99330), FP_to_Q15(1.45500)},
{FP_to_Q15(0.99387), FP_to_Q15(1.46000)},
{FP_to_Q15(0.99441), FP_to_Q15(1.46500)},
{FP_to_Q15(0.99492), FP_to_Q15(1.47000)},
{FP_to_Q15(0.99542), FP_to_Q15(1.47500)},
{FP_to_Q15(0.99588), FP_to_Q15(1.48000)},
{FP_to_Q15(0.99632), FP_to_Q15(1.48500)},
{FP_to_Q15(0.99674), FP_to_Q15(1.49000)},
{FP_to_Q15(0.99713), FP_to_Q15(1.49500)},
{FP_to_Q15(0.99749), FP_to_Q15(1.50000)},
{FP_to_Q15(0.99784), FP_to_Q15(1.50500)},
{FP_to_Q15(0.99815), FP_to_Q15(1.51000)},
{FP_to_Q15(0.99844), FP_to_Q15(1.51500)},
{FP_to_Q15(0.99871), FP_to_Q15(1.52000)},
{FP_to_Q15(0.99895), FP_to_Q15(1.52500)},
{FP_to_Q15(0.99917), FP_to_Q15(1.53000)},
{FP_to_Q15(0.99936), FP_to_Q15(1.53500)},
{FP_to_Q15(0.99953), FP_to_Q15(1.54000)},
{FP_to_Q15(0.99967), FP_to_Q15(1.54500)},
{FP_to_Q15(0.99978), FP_to_Q15(1.55000)},
{FP_to_Q15(0.99988), FP_to_Q15(1.55500)},
{FP_to_Q15(0.99994), FP_to_Q15(1.56000)},
{FP_to_Q15(0.99998), FP_to_Q15(1.56500)},
{FP_to_Q15(1.00000), FP_to_Q15(1.57000)},
};
