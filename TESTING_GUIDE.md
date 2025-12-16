# Testing Guide - Complete Authentication Flow

## 🔍 Debugging Steps

### Step 1: Clear Everything

1. **Sign out** if logged in (click your avatar → Sign Out)

2. **Clear browser completely:**
   - Open DevTools (F12)
   - Go to Application tab
   - Click "Clear site data" button
   - OR manually clear:
     - localStorage (delete all keys)
     - Cookies (delete all)
   - Close DevTools

3. **Hard refresh:**
   - `Ctrl + Shift + R` (Windows)
   - `Cmd + Shift + R` (Mac)

### Step 2: Test Signup Flow

1. Go to http://localhost:3000

2. Click **"Sign Up"** button in navbar

3. Fill the form:
   - **Name:** Test User
   - **Email:** test@example.com
   - **Password:** password123

4. Click **"Sign Up"**

5. **Watch the browser console** (F12 → Console tab)
   - You should see: `"Signup result:"` with the response
   - You should see: `"Showing level selection modal"`

6. **What should happen:**
   - ✅ Level selection modal appears
   - ✅ Shows: "Welcome, Test User! 👋"
   - ✅ Three level cards: Beginner, Intermediate, Advanced
   - ✅ Can select a level
   - ✅ "Continue" button becomes active

7. **Select a level** and click **"Continue"**

8. **What should happen:**
   - ✅ Redirects to homepage (/)
   - ✅ Shows personalized welcome banner
   - ✅ Shows your level badge
   - ✅ Shows recommended chapters

### Step 3: Check Profile Menu

1. Click on your **name/avatar** in navbar (top right)

2. **You should see:**
   - Your email
   - Your difficulty level with icon (🌱/⚡/🚀)
   - "Edit Profile" option
   - "Sign Out" option

3. Click **"Edit Profile"**

4. **You should see:**
   - Edit Profile modal
   - Your name and email (display only)
   - Level selector with current level selected
   - "Cancel" and "Save Changes" buttons

5. **Select different level** and click **"Save Changes"**

6. **What should happen:**
   - Modal closes
   - Page reloads
   - Homepage shows new level and recommendations

---

## 🐛 If Level Selection Doesn't Appear

### Check Browser Console

1. Press **F12** to open DevTools
2. Go to **Console** tab
3. Click "Sign Up" and complete the form
4. Look for these messages:

**Expected:**
```
Signup result: { ... }
Showing level selection modal
```

**If you see an error instead:**
- Copy the error message
- Check if it says "Email already exists" → Use a different email
- Check if auth server is running

### Check Network Tab

1. Press **F12** → **Network** tab
2. Click "Sign Up"
3. Look for request to `/api/auth/sign-up/email`
4. Click on it → Check:
   - **Status:** Should be 200 OK
   - **Response:** Should contain user data (not error)

### Verify Auth Server

1. Check Terminal 1 (auth-server)
2. Should see logs when you sign up
3. If no logs → auth server might have crashed

---

## 🔄 If Modal Appears But Nothing Happens

### Check Console After Clicking "Continue"

**Expected:**
```
Profile updated
Redirecting to homepage
```

### Check localStorage

1. F12 → Application tab → localStorage
2. Should see:
   - `user_profile` with your difficulty level
   - Key should contain: `{ "difficultyLevel": "intermediate", ... }`

---

## ✅ Complete Test Checklist

### Signup Flow
- [ ] Sign Up button works
- [ ] Form validation works (8+ char password)
- [ ] Signup submits successfully
- [ ] Level selection modal appears
- [ ] Can select a level
- [ ] Continue button becomes active after selection
- [ ] Redirects to homepage (/)

### Homepage
- [ ] Shows personalized greeting with name
- [ ] Shows time-based greeting (morning/afternoon/evening)
- [ ] Shows level badge with correct icon
- [ ] Shows level description
- [ ] Shows 2 recommended chapters
- [ ] Recommended chapters are correct for level

### Profile Menu
- [ ] Clicking avatar opens dropdown
- [ ] Shows email
- [ ] Shows difficulty level with icon
- [ ] Has "Edit Profile" option
- [ ] Has "Sign Out" option

### Edit Profile
- [ ] Modal opens when clicking "Edit Profile"
- [ ] Shows current name and email
- [ ] Current level is pre-selected
- [ ] Can change level
- [ ] "Save Changes" updates profile
- [ ] Homepage updates with new recommendations

### Login Flow (Existing User)
- [ ] Sign Out works
- [ ] Sign In button works
- [ ] Can log in with credentials
- [ ] Redirects to homepage
- [ ] Shows welcome banner
- [ ] Profile menu works

---

## 📝 Quick Troubleshooting

| Problem | Solution |
|---------|----------|
| Modal doesn't appear | Check console for errors, clear cache |
| "Continue" doesn't work | Check if level is selected, check console |
| No welcome on homepage | Check if user is authenticated, check localStorage |
| Profile menu empty | Check localStorage for `user_profile` |
| Recommendations wrong | Check localStorage, level might be wrong |
| Can't edit profile | Check if modal opens, check console for errors |

---

## 🎯 Expected Console Logs

### During Signup:
```javascript
Signup result: {
  data: { user: { id: "...", name: "Test User", email: "test@example.com" } }
}
Showing level selection modal
```

### During Level Selection:
```javascript
Saving level: intermediate
Profile updated: { difficultyLevel: "intermediate", onboardingComplete: true, ... }
```

### On Homepage:
```javascript
// No errors
// HomePage renders with user data
```

---

## 🚀 If Everything Fails

### Nuclear Option (Complete Reset):

1. **Kill both servers:**
   - Press `Ctrl+C` in both terminals

2. **Clear everything:**
   ```bash
   # Clear Docusaurus cache
   npm run clear
   ```

3. **Restart servers:**
   ```bash
   # Terminal 1
   cd auth-server
   npm run dev

   # Terminal 2 (from root)
   npm start
   ```

4. **Clear browser:**
   - Open Incognito/Private window
   - Go to http://localhost:3000

5. **Try signup flow again**

---

## ✅ Success Criteria

You'll know it's working when:

1. ✅ After signup → Level selection modal appears
2. ✅ After selecting level → Redirects to homepage
3. ✅ Homepage shows: "Good morning/afternoon/evening, [YourName]!"
4. ✅ Homepage shows your level badge (🌱/⚡/🚀)
5. ✅ Homepage shows 2 recommended chapters
6. ✅ Profile menu shows your level
7. ✅ Can edit profile and change level
8. ✅ Recommendations update when level changes

---

**If you still have issues, check the browser console and share the error messages!**
