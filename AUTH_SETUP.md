# Better Auth Setup - Simple Authentication

This project has Better Auth integrated for user authentication.

## ✅ What's Implemented

**Authentication Features:**
- ✅ Email/Password Signup
- ✅ Email/Password Signin
- ✅ Social OAuth (Google/GitHub) - configured but requires API keys
- ✅ User sessions with secure cookies
- ✅ Sign out functionality
- ✅ User menu with profile display

**Tech Stack:**
- **Backend**: Express.js with Better Auth
- **Database**: PostgreSQL (Neon)
- **Frontend**: React with Better Auth React client
- **UI Framework**: Docusaurus

## 🚀 Running the Project

### Start Both Servers

**Terminal 1 - Auth Server:**
```bash
cd auth-server
npm run dev
```
Server runs at: http://localhost:3001

**Terminal 2 - Docusaurus:**
```bash
npm start
```
Frontend runs at: http://localhost:3000

## 📝 Usage

### Sign Up
1. Navigate to http://localhost:3000
2. Click "Sign Up" button in navbar
3. Enter name, email, and password (min 8 characters)
4. Click "Sign Up"
5. You'll be logged in automatically

### Sign In
1. Click "Sign In" button
2. Enter email and password
3. Click "Sign In"

### Sign Out
1. Click on your name/avatar in the navbar
2. Select "Sign Out" from dropdown

## 📁 Project Structure

```
auth-server/
├── src/
│   ├── auth.ts          # Better Auth configuration
│   └── index.ts         # Express server
└── package.json

src/
├── components/
│   └── Auth/
│       ├── AuthButton.jsx   # Sign up/Sign in buttons
│       ├── LoginModal.jsx   # Login modal dialog
│       ├── SignupModal.jsx  # Signup modal dialog
│       ├── UserMenu.jsx     # User dropdown menu
│       └── styles.module.css
├── contexts/
│   └── AuthContext.jsx      # Auth state management
├── lib/
│   └── auth-client.js       # Better Auth client
└── theme/
    └── Root.js              # Auth provider wrapper
```

## 🔧 Configuration

### Environment Variables

**auth-server/.env:**
```env
DATABASE_URL=your_neon_postgres_url
BETTER_AUTH_SECRET=your_secret_key_32_chars_minimum
BETTER_AUTH_URL=http://localhost:3001
CORS_ORIGINS=http://localhost:3000
PORT=3001
```

**Optional - Social OAuth:**
```env
GOOGLE_CLIENT_ID=your_google_client_id
GOOGLE_CLIENT_SECRET=your_google_client_secret
GITHUB_CLIENT_ID=your_github_client_id
GITHUB_CLIENT_SECRET=your_github_client_secret
```

## 🗄️ Database

Better Auth automatically creates these tables:
- `user` - User accounts
- `session` - Active sessions
- `account` - OAuth accounts
- `verification` - Email verification tokens

## 🎨 Customization

### Change Auth Server URL

Edit `src/lib/auth-client.js`:
```javascript
const AUTH_SERVER_URLS = {
  development: "http://localhost:3001",
  production: "https://your-production-url.com",
};
```

### Modify UI Styling

Edit `src/components/Auth/styles.module.css` to customize:
- Modal appearance
- Button styles
- Form inputs
- Colors and spacing

## 🚢 Deployment

### Auth Server (Railway)
Already deployed at: `https://serene-mercy-production-5114.up.railway.app`

### Frontend (Vercel/Netlify)
Build command:
```bash
npm run build
```
Deploy the `build` directory.

## 📚 Resources

- [Better Auth Documentation](https://www.better-auth.com/)
- [Better Auth GitHub](https://github.com/better-auth/better-auth)

## 🐛 Troubleshooting

**Issue: CORS errors**
- Check `CORS_ORIGINS` in auth-server/.env
- Ensure frontend URL is included

**Issue: Session not persisting**
- Check browser allows cookies
- Verify `credentials: 'include'` in auth-client.js

**Issue: Social login not working**
- Add OAuth credentials to .env
- Configure OAuth app redirect URLs

---

**Status**: ✅ Production Ready
**Last Updated**: 2025-12-16
