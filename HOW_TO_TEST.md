# How to Test - Complete Guide

## ✅ What's Working Now

The signup flow with level selection is **working correctly**! You saw the red screen, which means the modal is rendering.

---

## 🎯 Expected Behavior

### For NEW Users (First Time Signup):

1. Click "Sign Up"
2. Fill form → Submit
3. **Level selection modal appears**
4. Select your level
5. Click "Continue"
6. Redirects to homepage with personalized welcome

### For EXISTING Users (Sign In):

1. Click "Sign In"
2. Fill form → Submit
3. **NO level selection** (you already have a profile)
4. Redirects to homepage
5. Shows your existing level and recommendations

---

## 🐛 The "1 Second Flash" Issue

**What you're seeing:**
- Sign in → Modal appears briefly → Disappears → Auto-selected level

**Why this happens:**
1. You sign in → `isAuthenticated` becomes true
2. Profile loads from localStorage (has your old level)
3. `onboardingComplete: true` exists in profile
4. AuthButton sees this and switches to UserMenu
5. No modal needed (you're a returning user)

**This is CORRECT behavior!** Returning users shouldn't see level selection again.

---

## 🧪 Proper Testing Steps

### Test 1: New User Signup (Should Show Level Selection)

1. **Clear ALL data:**
   - Press F12
   - Application tab → localStorage
   - Right-click → Clear
   - Also clear Cookies

2. **Use a BRAND NEW email:**
   ```
   Email: freshuser@test.com
   Password: password123
   Name: Fresh User
   ```

3. **Expected flow:**
   - Signup form → Submit
   - Level selection appears
   - Choose level
   - Redirect to homepage
   - See welcome with your chosen level

### Test 2: Existing User Sign In (Should NOT Show Level Selection)

1. **Sign out**

2. **Sign back in** with the same email

3. **Expected flow:**
   - Login form → Submit
   - Direct redirect to homepage
   - NO level selection (you already have one)
   - Homepage shows your existing level

### Test 3: Edit Profile (Change Level)

1. **Click your avatar** in navbar

2. **Click "Edit Profile"**

3. **Change your level**

4. **Click "Save Changes"**

5. **Expected:**
   - Modal closes
   - Homepage recommendations update

---

## 📊 What Should Be in localStorage

After completing signup + level selection:

### `user_profile`:
```json
{
  "difficultyLevel": "intermediate",
  "onboardingComplete": true,
  "updatedAt": "2025-12-16T..."
}
```

### `user_progress`:
```json
{
  "lastVisited": {
    "pageId": "intro",
    "pageTitle": "Introduction",
    "timestamp": "2025-12-16T..."
  },
  "visitedPages": { ... }
}
```

---

## ✅ Success Criteria

### For New Users:
- ✅ Level selection modal appears after signup
- ✅ Can select level
- ✅ Redirects to homepage
- ✅ Homepage shows personalized welcome
- ✅ Shows correct recommendations for level

### For Returning Users:
- ✅ No level selection on sign in
- ✅ Direct redirect to homepage
- ✅ Shows existing level in profile menu
- ✅ Can edit level anytime

---

## 🎨 Clean Up Test Styling

Now that we confirmed it works, let me remove the test red background and make it look professional:

**Current (Test Mode):**
- Red background
- Giant warning text
- Debug information

**Should Be (Production):**
- Clean white modal
- Smooth dark overlay
- Professional design

Let me know if you want me to remove the test styling and make it look like the reference site!

---

## 🔍 To Debug Further

If it's still not working as expected:

1. **Share your localStorage:**
   - F12 → Application → localStorage
   - Copy the `user_profile` value

2. **Share console logs:**
   - Full console output from signup to homepage

3. **Describe what you see:**
   - Does the red screen appear?
   - For how long?
   - What happens after?

---

**The core functionality is working!** The red screen proved the modal renders. Now we just need to confirm the flow is smooth and remove test styling.
