import json, numpy as np
R=json.load(open('features.json'))
def auroc(s,y):
    s=np.asarray(s,float);y=np.asarray(y,bool);p,n=s[y],s[~y]
    if not len(p) or not len(n): return float('nan')
    return float(((p[:,None]>n[None,:]).sum()+0.5*(p[:,None]==n[None,:]).sum())/(len(p)*len(n)))
def sp(a,b):
    ra=np.argsort(np.argsort(a));rb=np.argsort(np.argsort(b));return float(np.corrcoef(ra,rb)[0,1])
a=np.array([x['alpha_t'] for x in R]); al=np.array([x['alias_ratio'] for x in R]); cyl=np.array([x['cyl'] for x in R])
wrong=np.array([not x['correct'] for x in R]); same=np.array([x['gt_overlap']>=0.2 for x in R])
prog=np.array([x['progress_m'] for x in R]); run=np.array([x['run'] for x in R])
print('corr(alpha, alias) spearman', round(sp(a,al),3))
print('same-place subset n=%d wrong=%d'%(same.sum(),(wrong&same).sum()))
for nm,s in [('-alpha',-a),('alias',al),('-cyl',-cyl)]:
    print(f'  {nm}: AUROC wrong|same-place={auroc(s[same],wrong[same]):.3f}   AUROC diff-place(all)={auroc(s,~same):.3f}')
# position confound: is the pair inside the designed shared corridor? use same-place rate by progress bins
bins=np.arange(0,170,20)
print('progress bin: n, wrong-rate, same-place-rate, mean alpha, mean alias')
for lo in bins[:-1]:
    m=(prog>=lo)&(prog<lo+20)
    if m.sum(): print(f'  {lo:3d}-{lo+20:3d} m: {m.sum():3d}  {wrong[m].mean():.2f}  {same[m].mean():.2f}  {a[m].mean():.3f}  {al[m].mean():.3f}')
# residualize by position: AUROC within bins weighted
num=0;den=0
for lo in bins[:-1]:
    m=(prog>=lo)&(prog<lo+20)
    if wrong[m].sum() and (~wrong[m]).sum():
        k=wrong[m].sum()*(~wrong[m]).sum(); num+=auroc(-a[m],wrong[m])*k; den+=k
print('position-stratified AUROC -alpha', round(num/den,3) if den else None)
num=0;den=0
for lo in bins[:-1]:
    m=(prog>=lo)&(prog<lo+20)
    if wrong[m].sum() and (~wrong[m]).sum():
        k=wrong[m].sum()*(~wrong[m]).sum(); num+=auroc(al[m],wrong[m])*k; den+=k
print('position-stratified AUROC alias', round(num/den,3) if den else None)
# accepted (gate) vs wrong
acc=np.array([x['accepted'] for x in R]); print('gate accepted', acc.sum(), 'of which wrong', (acc&wrong).sum())
# alias offsets for wrong pairs
offs=[x['alias_off'] for x in R if x['alias_off']]
d=np.array([np.hypot(o[0],o[1]) for o in offs]); print('alias best-offset distance median %.1f m, IQR %.1f-%.1f'%(np.median(d),*np.percentile(d,[25,75])))
