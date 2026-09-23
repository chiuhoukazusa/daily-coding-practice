// Extended Kalman Filter (EKF) — Nonlinear State Estimation
//
// Canonical EKF problem: radar tracking. The target moves with constant velocity
// (linear dynamics), but the radar sensor returns RANGE and BEARING — both
// nonlinear functions of the Cartesian state [x, y, vx, vy]:
//       range   = sqrt(x^2 + y^2)
//       bearing = atan2(y, x)
//
// A standard (linear) Kalman filter requires a linear measurement model, so it
// must pick a FIXED linearization point (Jacobian H evaluated once, at the
// initial guess). The EKF re-linearizes H at the *current* estimate every step,
// which is the whole point of the "Extended" filter. On a large-range scenario
// the fixed linearization drifts and the plain KF diverges, while EKF tracks.
//
// Baselines (identical noisy range/bearing measurements):
//   1. "naive sensor": raw range/bearing -> Cartesian (no filtering). Its error
//      equals the measurement noise propagated to XY (~ range * bearing_noise).
//   2. "linear-KF (fixed H)": linear KF with H linearized once at t=0, then frozen.
//   3. "EKF (re-linearized H)": EKF with H re-evaluated every step.
//
// Quantitative verification:
//   1. EKF RMSE << naive-sensor RMSE.
//   2. EKF RMSE << fixed-H linear-KF RMSE (re-linearization advantage).
//   3. 3-sigma consistency >= 90%.

#include <cmath>
#include <cstdio>
#include <random>

static const int    N    = 3000;
static const double dt   = 0.05;      // 20 Hz
static const double sig_r = 2.0;      // range noise (m)
static const double sig_b = 0.008;    // bearing noise (rad)

int main() {
    std::mt19937 rng(2026);
    std::normal_distribution<double> gauss(0.0, 1.0);

    // Ground truth: target flies far in x so that the bearing-angle geometry
    // changes a lot over time => fixed linearization drifts badly.
    double tx = 800.0, ty = 600.0, tvx = -3.0, tvy = 1.5;

    // ---- EKF state/cov ----
    double x[4] = { tx + 10, ty - 10, tvx, tvy };
    double P[16] = {100,0,0,0, 0,100,0,0, 0,0,1,0, 0,0,0,1};

    // ---- Fixed-H linear KF (linearize once at a POOR initial guess) ----
    // Give it a badly-off initial state so the frozen linearization is wrong.
    double xf[4] = { x[0] + 300.0, x[1] - 300.0, x[2], x[3] };
    double Pf[16] = {100,0,0,0, 0,100,0,0, 0,0,1,0, 0,0,0,1};
    // Fixed H computed once at initial estimate:
    double r0 = std::hypot(x[0], x[1]);
    double Hf_rx = x[0]/r0, Hf_ry = x[1]/r0;
    double Hf_bx = -x[1]/(r0*r0), Hf_by = x[0]/(r0*r0);

    const double qv = 0.001;
    const double qx = qv*dt*dt;
    const double Rr = sig_r*sig_r, Rb = sig_b*sig_b;

    const int warmup = 300;
    double s_ekf=0, s_meas=0, s_lin=0;
    int in_bounds=0, count=0;

    for (int k=0;k<N;++k){
        // propagate truth
        tx += tvx*dt; ty += tvy*dt;
        double r = std::hypot(tx,ty), b = std::atan2(ty,tx);
        double zr = r + sig_r*gauss(rng);
        double zb = b + sig_b*gauss(rng);
        double mx = zr*std::cos(zb), my = zr*std::sin(zb); // naive Cartesian

        // ================= EKF =================
        // predict (linear CV)
        double xp[4] = { x[0]+x[2]*dt, x[1]+x[3]*dt, x[2], x[3] };
        double ap[16];
        ap[0]=P[0]+dt*P[8]; ap[1]=P[1]+dt*P[9]; ap[2]=P[2]+dt*P[10]; ap[3]=P[3]+dt*P[11];
        ap[4]=P[4]+dt*P[12]; ap[5]=P[5]+dt*P[13]; ap[6]=P[6]+dt*P[14]; ap[7]=P[7]+dt*P[15];
        ap[8]=P[8]; ap[9]=P[9]; ap[10]=P[10]; ap[11]=P[11];
        ap[12]=P[12]; ap[13]=P[13]; ap[14]=P[14]; ap[15]=P[15];
        double Pp[16];
        Pp[0]=ap[0]+dt*ap[2]; Pp[1]=ap[1]+dt*ap[3]; Pp[2]=ap[2]; Pp[3]=ap[3];
        Pp[4]=ap[4]+dt*ap[6]; Pp[5]=ap[5]+dt*ap[7]; Pp[6]=ap[6]; Pp[7]=ap[7];
        Pp[8]=ap[8]+dt*ap[10]; Pp[9]=ap[9]+dt*ap[11]; Pp[10]=ap[10]; Pp[11]=ap[11];
        Pp[12]=ap[12]+dt*ap[14]; Pp[13]=ap[13]+dt*ap[15]; Pp[14]=ap[14]; Pp[15]=ap[15];
        Pp[0]+=qx; Pp[5]+=qx; Pp[10]+=qv; Pp[15]+=qv;

        // update with re-linearized Jacobian at current prediction
        double pr = std::hypot(xp[0],xp[1]);
        double pb = std::atan2(xp[1],xp[0]);
        double pr2 = pr*pr;
        double Hr[4] = { xp[0]/pr, xp[1]/pr, 0, 0 };
        double Hb[4] = { -xp[1]/pr2, xp[0]/pr2, 0, 0 };
        double dzr = zr - pr;
        double dzb = zb - pb;
        while (dzb >  M_PI) dzb -= 2*M_PI;
        while (dzb < -M_PI) dzb += 2*M_PI;
        // innovation covariance S = H P H^T + R
        double hpr[4] = {  // P Hr^T
            Pp[0]*Hr[0]+Pp[1]*Hr[1], Pp[4]*Hr[0]+Pp[5]*Hr[1],
            Pp[8]*Hr[0]+Pp[9]*Hr[1], Pp[12]*Hr[0]+Pp[13]*Hr[1] };
        double hpb[4] = {  // P Hb^T
            Pp[0]*Hb[0]+Pp[1]*Hb[1], Pp[4]*Hb[0]+Pp[5]*Hb[1],
            Pp[8]*Hb[0]+Pp[9]*Hb[1], Pp[12]*Hb[0]+Pp[13]*Hb[1] };
        double s_rr = Hr[0]*hpr[0]+Hr[1]*hpr[1]+Rr;
        double s_bb = Hb[0]*hpb[0]+Hb[1]*hpb[1]+Rb;
        double s_rb = Hr[0]*hpb[0]+Hr[1]*hpb[1];
        double det = s_rr*s_bb - s_rb*s_rb;
        double si_rr = s_bb/det, si_rb = -s_rb/det, si_bb = s_rr/det;
        // K = P H^T S^-1
        double K[8];
        for (int i=0;i<4;i++){
            K[2*i]   = hpr[i]*si_rr + hpb[i]*si_rb;
            K[2*i+1] = hpr[i]*si_rb + hpb[i]*si_bb;
        }
        x[0]=xp[0]+K[0]*dzr+K[1]*dzb;
        x[1]=xp[1]+K[2]*dzr+K[3]*dzb;
        x[2]=xp[2]+K[4]*dzr+K[5]*dzb;
        x[3]=xp[3]+K[6]*dzr+K[7]*dzb;
        // P = (I - KH) Pp ; KH = K*H
        double KH[16];
        for (int rw=0;rw<4;rw++){
            KH[rw*4+0]=K[2*rw]*Hr[0]+K[2*rw+1]*Hb[0];
            KH[rw*4+1]=K[2*rw]*Hr[1]+K[2*rw+1]*Hb[1];
            KH[rw*4+2]=0; KH[rw*4+3]=0;
        }
        double Pnew[16];
        for (int i=0;i<16;i++){
            double sum=0;
            for (int c=0;c<4;c++){
                double ikhc = (i/4==c)?1.0:0.0;
                sum += (ikhc - KH[(i/4)*4+c]) * Pp[c*4 + (i%4)];
            }
            Pnew[i]=sum;
        }
        for (int i=0;i<16;i++) P[i]=Pnew[i];

        // ================= Fixed-H linear KF =================
        {
            double xfp[4]={xf[0]+xf[2]*dt, xf[1]+xf[3]*dt, xf[2], xf[3]};
            double af[16];
            af[0]=Pf[0]+dt*Pf[8]; af[1]=Pf[1]+dt*Pf[9]; af[2]=Pf[2]+dt*Pf[10]; af[3]=Pf[3]+dt*Pf[11];
            af[4]=Pf[4]+dt*Pf[12]; af[5]=Pf[5]+dt*Pf[13]; af[6]=Pf[6]+dt*Pf[14]; af[7]=Pf[7]+dt*Pf[15];
            af[8]=Pf[8]; af[9]=Pf[9]; af[10]=Pf[10]; af[11]=Pf[11];
            af[12]=Pf[12]; af[13]=Pf[13]; af[14]=Pf[14]; af[15]=Pf[15];
            double Pfp[16];
            Pfp[0]=af[0]+dt*af[2]; Pfp[1]=af[1]+dt*af[3]; Pfp[2]=af[2]; Pfp[3]=af[3];
            Pfp[4]=af[4]+dt*af[6]; Pfp[5]=af[5]+dt*af[7]; Pfp[6]=af[6]; Pfp[7]=af[7];
            Pfp[8]=af[8]+dt*af[10]; Pfp[9]=af[9]+dt*af[11]; Pfp[10]=af[10]; Pfp[11]=af[11];
            Pfp[12]=af[12]+dt*af[14]; Pfp[13]=af[13]+dt*af[15]; Pfp[14]=af[14]; Pfp[15]=af[15];
            Pfp[0]+=qx; Pfp[5]+=qx; Pfp[10]+=qv; Pfp[15]+=qv;
            // frozen Jacobian Hf
            double pr_f = std::hypot(xfp[0],xfp[1]);
            double pb_f = std::atan2(xfp[1],xfp[0]);
            double dzr_f = zr - pr_f;
            double dzb_f = zb - pb_f;
            while (dzb_f >  M_PI) dzb_f -= 2*M_PI;
            while (dzb_f < -M_PI) dzb_f += 2*M_PI;
            double hfr[4]={Pfp[0]*Hf_rx+Pfp[1]*Hf_ry, Pfp[4]*Hf_rx+Pfp[5]*Hf_ry,
                           Pfp[8]*Hf_rx+Pfp[9]*Hf_ry, Pfp[12]*Hf_rx+Pfp[13]*Hf_ry};
            double hfb[4]={Pfp[0]*Hf_bx+Pfp[1]*Hf_by, Pfp[4]*Hf_bx+Pfp[5]*Hf_by,
                           Pfp[8]*Hf_bx+Pfp[9]*Hf_by, Pfp[12]*Hf_bx+Pfp[13]*Hf_by};
            double srr=Hf_rx*hfr[0]+Hf_ry*hfr[1]+Rr;
            double sbb=Hf_bx*hfb[0]+Hf_by*hfb[1]+Rb;
            double srb=Hf_rx*hfb[0]+Hf_ry*hfb[1];
            double detf=srr*sbb-srb*srb;
            double cirr=sbb/detf, cirb=-srb/detf, cibb=srr/detf;
            double Kf[8];
            for (int i=0;i<4;i++){
                Kf[2*i]  =hfr[i]*cirr+hfb[i]*cirb;
                Kf[2*i+1]=hfr[i]*cirb+hfb[i]*cibb;
            }
            xf[0]=xfp[0]+Kf[0]*dzr_f+Kf[1]*dzb_f;
            xf[1]=xfp[1]+Kf[2]*dzr_f+Kf[3]*dzb_f;
            xf[2]=xfp[2]+Kf[4]*dzr_f+Kf[5]*dzb_f;
            xf[3]=xfp[3]+Kf[6]*dzr_f+Kf[7]*dzb_f;
            // P update with frozen H
            double KHf[16];
            for (int rw=0;rw<4;rw++){
                KHf[rw*4+0]=Kf[2*rw]*Hf_rx+Kf[2*rw+1]*Hf_bx;
                KHf[rw*4+1]=Kf[2*rw]*Hf_ry+Kf[2*rw+1]*Hf_by;
                KHf[rw*4+2]=0; KHf[rw*4+3]=0;
            }
            for (int i=0;i<16;i++){
                double sum=0;
                for (int c=0;c<4;c++){
                    double ikhc=(i/4==c)?1.0:0.0;
                    sum+=(ikhc-KHf[(i/4)*4+c])*Pfp[c*4+(i%4)];
                }
                Pf[i]=sum;
            }
        }

        if (k>=warmup){
            double e_ekf = std::hypot(x[0]-tx,x[1]-ty);
            double e_meas= std::hypot(mx-tx,my-ty);
            double e_lin = std::hypot(xf[0]-tx,xf[1]-ty);
            s_ekf+=e_ekf*e_ekf; s_meas+=e_meas*e_meas; s_lin+=e_lin*e_lin;
            double sigma=std::sqrt(P[0]+P[5]);
            if (std::hypot(x[0]-tx,x[1]-ty)<=3.0*sigma) ++in_bounds;
            ++count;
        }
    }

    double rmse_ekf = std::sqrt(s_ekf/count);
    double rmse_meas= std::sqrt(s_meas/count);
    double rmse_lin = std::sqrt(s_lin/count);
    double ratio=100.0*in_bounds/count;

    printf("============ EKF Radar Tracking (Nonlinear Measurement) ============\n");
    printf("Dynamics: constant velocity (linear).  Sensor: range+bearing (nonlinear).\n");
    printf("range std=%.1fm  bearing std=%.3frad   target flies %.0fm\n", sig_r, sig_b, std::hypot(tx,ty));
    printf("Steps: %d (post-warmup %d, dt=%.2fs)\n", N, count, dt);
    printf("--------------------------------------------------------------------\n");
    printf("RMSE(position)  EKF (re-linearized H) : %.4f m\n", rmse_ekf);
    printf("RMSE(position)  naive sensor          : %.4f m\n", rmse_meas);
    printf("RMSE(position)  linear-KF (frozen H)  : %.4f m\n", rmse_lin);
    printf("--------------------------------------------------------------------\n");
    printf("EKF / naive sensor  : %.2fx\n", rmse_meas/rmse_ekf);
    printf("EKF / frozen-H KF   : %.2fx\n", rmse_lin/rmse_ekf);
    printf("3-sigma consistency (target >=90%%): %.2f%%\n", ratio);
    printf("====================================================================\n");

    bool pass=true;
    if (!(rmse_ekf < rmse_meas*0.7)) { printf("FAIL: EKF not clearly below naive sensor\n"); pass=false; }
    if (!(rmse_ekf < rmse_lin))   { printf("FAIL: EKF not below frozen-H linear KF\n"); pass=false; }
    if (!(ratio>=90.0))            { printf("FAIL: consistency<90%%\n"); pass=false; }
    printf(pass?"RESULT: PASS\n":"RESULT: FAIL\n");
    return pass?0:1;
}
