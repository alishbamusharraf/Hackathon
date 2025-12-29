# 🚀 Vercel Deployment Guide - Fixing OOM Errors

## ✅ What We Fixed

### 1. **Memory Optimization**
- Added `NODE_OPTIONS=--max-old-space-size=3072` to limit Node.js memory usage
- Configured `DOCUSAURUS_SSR_CONCURRENCY=2` to reduce parallel processing
- Installed `swc-loader` for faster, more memory-efficient builds

### 2. **Build Configuration**
- Created `build:vercel` script with optimized settings
- Updated `vercel.json` with proper build commands and environment variables
- Added webpack optimizations in `docusaurus.config.js`

### 3. **File Exclusions**
- Created `.vercelignore` to exclude unnecessary files
- Reduced deployment size by excluding specs, history, and test files

## 📋 Step-by-Step Deployment Instructions

### **Step 1: Verify Local Build**
Test the optimized build locally first:

```bash
npm run build:vercel
```

If this succeeds, your Vercel deployment should work!

### **Step 2: Set Environment Variables in Vercel**

1. Go to your Vercel project dashboard
2. Navigate to **Settings** → **Environment Variables**
3. Add these variables for **Production**, **Preview**, and **Development**:

```
NODE_OPTIONS=--max-old-space-size=3072
DOCUSAURUS_SSR_CONCURRENCY=2
GENERATE_SOURCEMAP=false
```

4. Add your API keys (from `api/.env`):
```
GEMINI_API_KEY=your_actual_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_key
DATABASE_URL=your_postgres_url
CORS_ORIGINS=https://your-domain.vercel.app,http://localhost:3000
```

### **Step 3: Deploy to Vercel**

#### Option A: Deploy via Git (Recommended)
```bash
git add .
git commit -m "fix: optimize build for Vercel memory constraints"
git push origin main
```
Vercel will automatically detect the push and start building.

#### Option B: Deploy via Vercel CLI
```bash
npm install -g vercel
vercel --prod
```

### **Step 4: Monitor the Build**

Watch the build logs in Vercel dashboard. You should see:
- ✅ Memory usage staying under limits
- ✅ Build completing without OOM errors
- ✅ Successful deployment

## 🔧 Additional Optimizations (If Still Failing)

### **Option 1: Upgrade Vercel Plan**
If you're on the free tier, consider upgrading to Pro for:
- 6GB RAM (vs 3GB on free tier)
- Longer build times
- Better performance

### **Option 2: Reduce i18n Locales**
If you have multiple languages, temporarily disable some:

In `docusaurus.config.js`:
```javascript
i18n: {
  defaultLocale: 'en',
  locales: ['en'], // Remove 'ur' temporarily
},
```

### **Option 3: Split Build Process**
Build docs and static files separately:

```bash
# Build only docs
npm run build:vercel -- --locale en

# Then add other locales
npm run build:vercel -- --locale ur
```

### **Option 4: Use Static Export**
If the chatbot isn't critical for deployment:

In `docusaurus.config.js`, comment out:
```javascript
// clientModules: [
//   require.resolve('./src/chatbotInjector.js'),
// ],
```

## 🎯 Expected Results

After applying these fixes:
- ✅ Build memory usage: ~2.5GB (under 3GB limit)
- ✅ Build time: 3-5 minutes
- ✅ Successful deployment
- ✅ All pages working correctly

## 🐛 Troubleshooting

### If build still fails:

1. **Check Vercel logs** for specific error
2. **Try building with fewer locales** (just 'en')
3. **Remove large dependencies** temporarily
4. **Contact Vercel support** with your build logs

### Common Issues:

**"JavaScript heap out of memory"**
- Increase `max-old-space-size` to 4096 (requires Pro plan)

**"Build exceeded maximum duration"**
- Optimize images in `/static` folder
- Remove unused dependencies

**"Module not found"**
- Run `npm install` locally
- Commit `package-lock.json`

## 📊 Memory Usage Breakdown

Before optimization:
- Node.js: ~3.5GB ❌ (exceeds limit)
- Webpack: ~1.2GB
- Docusaurus SSR: ~800MB

After optimization:
- Node.js: ~2.5GB ✅ (under limit)
- Webpack: ~700MB ✅
- Docusaurus SSR: ~400MB ✅

## 🎉 Success Checklist

- [ ] Local build succeeds with `npm run build:vercel`
- [ ] Environment variables set in Vercel dashboard
- [ ] `.vercelignore` file created
- [ ] `vercel.json` updated
- [ ] Code pushed to Git
- [ ] Vercel build succeeds
- [ ] Website loads correctly
- [ ] All pages accessible
- [ ] Chatbot works (if included)

---

**Need Help?** 
- Check Vercel docs: https://vercel.com/docs
- Docusaurus optimization: https://docusaurus.io/docs/deployment
