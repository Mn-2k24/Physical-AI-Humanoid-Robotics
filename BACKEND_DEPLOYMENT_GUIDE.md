# Backend Deployment Guide - Gemini API Fix

**Date**: 2025-12-24
**Tasks**: T146-T154 (Phase 13)

## 🎯 What Was Fixed

### Root Cause Identified (T151):
- **Wrong Gemini model name**: `gemini-2.0-flash-exp` → `gemini-2.5-flash`
- The experimental model was causing rate limit errors
- Model name didn't match specification requirements

### Fixes Implemented (T152-T153):
1. ✅ Updated model name in `src/services/rag.py` (line 123)
2. ✅ Added comprehensive error handling with detailed logging
3. ✅ Distinguished between different error types (rate limit, auth, model not found)
4. ✅ Added usage metadata logging for debugging
5. ✅ Enhanced error messages with actionable troubleshooting steps
6. ✅ Created test script: `scripts/test_gemini_fix.py`

### Files Modified:
- `src/services/rag.py` - Fixed model name and enhanced error handling
- `scripts/test_gemini_fix.py` - New test script

### Git Status:
```
✅ Committed: 73f5286 - "Fix Gemini API rate limit error - Update model to gemini-2.5-flash"
⏳ Pending: Push to Hugging Face (token expired)
```

---

## 🚀 How to Deploy to Hugging Face

### Option 1: Using Git (Recommended)

1. **Generate a new Hugging Face token**:
   - Go to: https://huggingface.co/settings/tokens
   - Click "New token"
   - Name: "backend-deployment"
   - Type: "Write" access
   - Copy the token

2. **Navigate to backend folder**:
   ```bash
   cd /home/nizam/projects/Physical-AI-Humanoid-Robotics/physical-ai-humanoid-robotics-backend
   ```

3. **Check current status**:
   ```bash
   git status
   git log -1 --oneline
   ```

4. **Push to Hugging Face** (replace `YOUR_NEW_TOKEN` with actual token):
   ```bash
   git remote set-url origin https://Mn-2k24:YOUR_NEW_TOKEN@huggingface.co/spaces/Mn-2k24/physical-ai-humanoid-robotics-backend
   git push origin main
   ```

5. **Verify deployment**:
   - Go to: https://huggingface.co/spaces/Mn-2k24/physical-ai-humanoid-robotics-backend
   - Check logs for startup messages
   - Look for: "RAG Service initialized with model: gemini-2.5-flash"

### Option 2: Manual File Upload

If git push continues to fail:

1. Go to your Hugging Face Space:
   https://huggingface.co/spaces/Mn-2k24/physical-ai-humanoid-robotics-backend

2. Navigate to `Files` tab

3. Upload modified files:
   - Upload `src/services/rag.py`
   - Upload `scripts/test_gemini_fix.py`

4. The Space will automatically rebuild and redeploy

---

## 🔍 Verify the Fix

### 1. Check Hugging Face Logs

After deployment, monitor the logs:

```
Expected startup logs:
✓ RAG Service initialized with model: gemini-2.5-flash
✓ API Key prefix: AIzaSy...
```

### 2. Test from Frontend

1. Go to your frontend: https://physical-ai-humanoid-robotics-zeta.vercel.app
2. Sign in to your account
3. Ask a question in the chatbot
4. **Expected result**: Response returned successfully (no rate limit error)

### 3. Monitor for Errors

Check Hugging Face Space logs for:
- ✅ **Success**: "✓ Generated answer in XXXms for query..."
- ❌ **Rate Limit**: "⚠️  RATE LIMIT ERROR DETECTED"
- ❌ **Auth Error**: "⚠️  AUTHENTICATION ERROR DETECTED"
- ❌ **Model Error**: "⚠️  MODEL NOT FOUND ERROR"

---

## 🔧 If Rate Limits Still Occur

If you still see rate limit errors after deploying the fix, check:

### 1. Verify GEMINI_API_KEY is Updated in Hugging Face

1. Go to your Space settings:
   https://huggingface.co/spaces/Mn-2k24/physical-ai-humanoid-robotics-backend/settings

2. Check "Secrets" section

3. Verify `GEMINI_API_KEY` is set to your **new** API key

4. If you changed the key recently, restart the Space:
   - Settings → Factory Reboot

### 2. Check Gemini API Quota

1. Go to Google AI Studio: https://makersuite.google.com/app/apikey

2. Check your API key quotas:
   - **Free tier**: 60 requests per minute, 1500 per day
   - **Paid tier**: Higher limits

3. If quota is exhausted:
   - Wait for quota reset (24 hours for daily limit)
   - Or upgrade to paid tier
   - Or create a new API key

### 3. Test Locally First

Before deploying, you can test locally:

```bash
cd /home/nizam/projects/Physical-AI-Humanoid-Robotics/physical-ai-humanoid-robotics-backend

# Make sure .env has GEMINI_API_KEY set
# Then run test script:
python scripts/test_gemini_fix.py
```

---

## 📊 Summary of Changes

| File | Change | Purpose |
|------|--------|---------|
| `src/services/rag.py:123` | `gemini-2.0-flash-exp` → `gemini-2.5-flash` | Fix model name to match spec |
| `src/services/rag.py:126-127` | Added initialization logging | Debug API key and model |
| `src/services/rag.py:269-272` | Added request logging | Track generation details |
| `src/services/rag.py:280-282` | Added usage metadata logging | Monitor API usage |
| `src/services/rag.py:304-341` | Enhanced error handling | Provide actionable error messages |
| `scripts/test_gemini_fix.py` | New file | Test and verify the fix |

---

## 🎯 Expected Outcome

After successful deployment:

1. ✅ Chatbot returns responses successfully
2. ✅ No "Rate limit exceeded" errors (unless actual quota exhausted)
3. ✅ Detailed logs help identify any remaining issues
4. ✅ Error messages provide clear troubleshooting steps

---

## 📞 Need Help?

If issues persist after deployment:

1. Check Hugging Face Space logs for specific error messages
2. Verify all environment variables are set correctly
3. Test the Gemini API key directly: https://makersuite.google.com/app/apikey
4. Share the exact error message from logs for further debugging

---

**Status**: ✅ Code fixes ready, ⏳ Awaiting deployment with valid HF token

**Next Steps**:
1. Generate new Hugging Face token
2. Push changes using git (Option 1) or upload files manually (Option 2)
3. Verify deployment
4. Test chatbot from frontend
